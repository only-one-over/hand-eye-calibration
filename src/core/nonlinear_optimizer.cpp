#include "core/nonlinear_optimizer.h"

#include "core/calibration_service.h"
#include "core/matrix_utils.h"
#include "core/normalized_huber.h"
#include "core/pose_quality_service.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>

namespace handeye {

namespace {

struct State {
    cv::Matx44d pose = cv::Matx44d::eye();
};

struct ResidualSet {
    QVector<cv::Vec3d> rotation;
    QVector<cv::Vec3d> translation;
    QVector<double> normalizedScores;
    QVector<double> weights;
    double loss = 0.0;
};

cv::Vec3d translationOf(const cv::Matx44d &pose)
{
    return {pose(0, 3), pose(1, 3), pose(2, 3)};
}

ResidualSet residuals(const CalibrationDataset &dataset, const State &state)
{
    CalibrationDataset current = dataset;
    const FixedTargetPoseReport mean = PoseQualityService::computeFixedTargetPose(
        current, matrix::toArray(state.pose));
    const cv::Matx44d reference = matrix::toMat(mean.robustMeanPose);
    ResidualSet result;
    for (const PoseSample &sample : dataset.samples) {
        const cv::Matx44d gripper = matrix::fromRodrigues(sample.gripperRotation,
                                                           sample.gripperTranslation);
        const cv::Matx44d target = matrix::fromRodrigues(sample.targetRotation,
                                                         sample.targetTranslation);
        const cv::Matx44d predicted = gripper * state.pose * target;
        const cv::Vec3d translation = translationOf(predicted) - translationOf(reference);
        const Vector3 rotationVector = matrix::toRodrigues(reference.get_minor<3, 3>(0, 0).t()
                                                            * predicted.get_minor<3, 3>(0, 0));
        const cv::Vec3d rotation(rotationVector[0], rotationVector[1], rotationVector[2]);
        result.rotation.append(rotation);
        result.translation.append(translation);
        const NormalizedHuberEvaluation evaluation = NormalizedHuber::evaluate(
            {rotation[0], rotation[1], rotation[2]},
            {translation[0], translation[1], translation[2]});
        result.normalizedScores.append(evaluation.norm);
        result.weights.append(evaluation.weight);
        result.loss += evaluation.loss;
    }
    return result;
}

void fillReportFromFixedPose(const FixedTargetPoseReport &fixed,
                             CalibrationResult *result)
{
    result->fixedTargetReport = fixed;
}

} // namespace

CalibrationResult NonlinearOptimizer::refinePose(const CalibrationDataset &dataset,
                                                 const CalibrationResult &seed,
                                                 int maxIterations)
{
    CalibrationResult result = seed;
    result.method = CalibrationMethod::Nonlinear;
    result.seedMethod = seed.method == CalibrationMethod::Nonlinear ? seed.seedMethod : seed.method;
    result.recommended = false;
    result.qualityReport = PoseQualityService::evaluatePoseQuality(dataset);
    result.optimizationReport = {};
    result.optimizationReport.available = true;
    if (!seed.success || dataset.inputMode != CalibrationInputMode::PosePairs) {
        result.success = false;
        result.message = QStringLiteral("非线性精修需要一个成功的 PosePairs 算法结果。");
        return result;
    }

    State state;
    state.pose = matrix::toMat(seed.cameraToGripper);
    const FixedTargetPoseReport before = PoseQualityService::computeFixedTargetPose(
        dataset, seed.cameraToGripper);
    result.optimizationReport.beforeRotationRmseDeg = before.rotationRmseDeg;
    result.optimizationReport.beforeTranslationRmseM = before.translationRmseM;
    const ResidualSet initialResiduals = residuals(dataset, state);
    result.optimizationReport.normalizedHuberLossBefore = initialResiduals.loss;
    double currentLoss = initialResiduals.loss;
    double lambda = 1e-3;
    for (int iteration = 0; iteration < std::max(1, maxIterations); ++iteration) {
        const ResidualSet base = residuals(dataset, state);
        cv::Mat hessian = cv::Mat::zeros(6, 6, CV_64F);
        cv::Mat gradient = cv::Mat::zeros(6, 1, CV_64F);
        for (int sampleIndex = 0; sampleIndex < dataset.samples.size(); ++sampleIndex) {
            cv::Mat jacobian(6, 6, CV_64F, cv::Scalar(0));
            const double values[6] = {
                base.rotation.at(sampleIndex)[0] / (CV_PI / 180.0),
                base.rotation.at(sampleIndex)[1] / (CV_PI / 180.0),
                base.rotation.at(sampleIndex)[2] / (CV_PI / 180.0),
                base.translation.at(sampleIndex)[0] / 0.001,
                base.translation.at(sampleIndex)[1] / 0.001,
                base.translation.at(sampleIndex)[2] / 0.001};
            for (int parameter = 0; parameter < 6; ++parameter) {
                State perturbed = state;
                const double step = parameter < 3 ? 1e-6 : 1e-7;
                Vector3 rotation = matrix::toRodrigues(perturbed.pose.get_minor<3, 3>(0, 0));
                Vector3 translation{perturbed.pose(0, 3), perturbed.pose(1, 3), perturbed.pose(2, 3)};
                if (parameter < 3) rotation[parameter] += step;
                else translation[parameter - 3] += step;
                perturbed.pose = matrix::fromRodrigues(rotation, translation);
                const ResidualSet changed = residuals(dataset, perturbed);
                const double changedValues[6] = {
                    changed.rotation.at(sampleIndex)[0] / (CV_PI / 180.0),
                    changed.rotation.at(sampleIndex)[1] / (CV_PI / 180.0),
                    changed.rotation.at(sampleIndex)[2] / (CV_PI / 180.0),
                    changed.translation.at(sampleIndex)[0] / 0.001,
                    changed.translation.at(sampleIndex)[1] / 0.001,
                    changed.translation.at(sampleIndex)[2] / 0.001};
                for (int component = 0; component < 6; ++component)
                    jacobian.at<double>(component, parameter) = (changedValues[component] - values[component]) / step;
            }
            cv::Mat residual(6, 1, CV_64F);
            for (int component = 0; component < 6; ++component) residual.at<double>(component, 0) = values[component];
            for (int component = 0; component < 6; ++component) {
                const double weight = base.weights.at(sampleIndex);
                const cv::Mat row = jacobian.row(component);
                hessian += weight * row.t() * row;
                gradient += weight * row.t() * residual.row(component).t();
            }
        }
        for (int diagonal = 0; diagonal < 6; ++diagonal)
            hessian.at<double>(diagonal, diagonal) += lambda * (hessian.at<double>(diagonal, diagonal) + 1e-9);
        cv::Mat delta;
        if (!cv::solve(hessian, -gradient, delta, cv::DECOMP_SVD)) break;
        State trial = state;
        Vector3 trialRotation = matrix::toRodrigues(trial.pose.get_minor<3, 3>(0, 0));
        Vector3 trialTranslation{trial.pose(0, 3), trial.pose(1, 3), trial.pose(2, 3)};
        for (int index = 0; index < 3; ++index) trialRotation[index] += delta.at<double>(index);
        for (int index = 0; index < 3; ++index) trialTranslation[index] += delta.at<double>(index + 3);
        trial.pose = matrix::fromRodrigues(trialRotation, trialTranslation);
        const double trialLoss = residuals(dataset, trial).loss;
        if (trialLoss < currentLoss) {
            const double previousLoss = currentLoss;
            state = trial;
            currentLoss = trialLoss;
            lambda = std::max(lambda * 0.5, 1e-9);
            result.optimizationReport.iterations = iteration + 1;
            const bool tinyStep = cv::norm(delta) < 1e-8;
            const bool tinyLossChange = std::abs(previousLoss - trialLoss)
                                        <= 1e-8 * std::max(1.0, std::abs(previousLoss));
            if (tinyStep || tinyLossChange) {
                result.optimizationReport.converged = true;
                break;
            }
        } else {
            lambda = std::min(lambda * 4.0, 1e8);
        }
    }

    result.cameraToGripper = matrix::toArray(state.pose);
    const FixedTargetPoseReport after = PoseQualityService::computeFixedTargetPose(
        dataset, result.cameraToGripper);
    fillReportFromFixedPose(after, &result);
    result.axXbReport = CalibrationService::evaluateAxXb(dataset, result.cameraToGripper);
    result.trainingReport = result.axXbReport;
    result.rotationErrorDeg = result.axXbReport.rotationRmseDeg;
    result.translationError = result.axXbReport.translationRmseM;
    result.optimizationReport.afterRotationRmseDeg = after.rotationRmseDeg;
    result.optimizationReport.afterTranslationRmseM = after.translationRmseM;
    const ResidualSet finalResiduals = residuals(dataset, state);
    result.optimizationReport.normalizedHuberLossAfter = finalResiduals.loss;
    result.optimizationReport.success = result.axXbReport.valid && after.success;
    result.optimizationReport.huberOutlierCount = 0;
    for (double score : finalResiduals.normalizedScores)
        result.optimizationReport.huberOutlierCount += score > 1.0 ? 1 : 0;
    result.success = result.axXbReport.valid && after.success;
    result.message = result.optimizationReport.converged
                         ? (result.axXbReport.passed ? QStringLiteral("非线性精修收敛，AX=XB 通过。")
                                                       : QStringLiteral("非线性精修收敛，但 AX=XB 未通过。"))
                         : QStringLiteral("非线性精修完成，但未达到严格收敛条件。");
    return result;
}

} // namespace handeye
