#include "core/nonlinear_optimizer.h"

#include "core/matrix_utils.h"
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
    double loss = 0.0;
};

cv::Vec3d translationOf(const cv::Matx44d &pose)
{
    return {pose(0, 3), pose(1, 3), pose(2, 3)};
}

double norm3(const cv::Vec3d &value)
{
    return std::sqrt(value.dot(value));
}

double huberLoss(double value, double threshold)
{
    const double absolute = std::abs(value);
    if (absolute <= threshold) return 0.5 * value * value;
    return threshold * (absolute - 0.5 * threshold);
}

double huberWeight(double value, double threshold)
{
    const double absolute = std::abs(value);
    return absolute <= threshold ? 1.0 : threshold / std::max(absolute, 1e-12);
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
        for (int component = 0; component < 3; ++component) {
            result.loss += huberLoss(rotation[component], CV_PI / 180.0);
            result.loss += huberLoss(translation[component], 0.001);
        }
    }
    return result;
}

void fillReportFromFixedPose(const FixedTargetPoseReport &fixed,
                             CalibrationResult *result)
{
    result->fixedTargetReport = fixed;
    result->trainingReport.available = fixed.success;
    result->trainingReport.valid = fixed.success;
    result->trainingReport.sampleCount = fixed.samples.size();
    result->trainingReport.outlierCount = fixed.outlierCount;
    result->trainingReport.rotationRmseDeg = fixed.rotationRmseDeg;
    result->trainingReport.translationRmseM = fixed.translationRmseM;
    result->trainingReport.rotationMeanDeg = fixed.rotationMeanDeg;
    result->trainingReport.translationMeanM = fixed.translationMeanM;
    result->trainingReport.rotationMaxDeg = fixed.rotationMaxDeg;
    result->trainingReport.translationMaxM = fixed.translationMaxM;
    result->trainingReport.passed = fixed.success && fixed.outlierCount == 0
                                    && fixed.rotationRmseDeg <= 0.5
                                    && fixed.translationRmseM <= 0.001;
    result->rotationErrorDeg = fixed.rotationRmseDeg;
    result->translationError = fixed.translationRmseM;
}

} // namespace

CalibrationResult NonlinearOptimizer::refinePose(const CalibrationDataset &dataset,
                                                 const CalibrationResult &seed)
{
    CalibrationResult result = seed;
    result.method = CalibrationMethod::Nonlinear;
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
    double currentLoss = residuals(dataset, state).loss;
    double lambda = 1e-3;
    for (int iteration = 0; iteration < 50; ++iteration) {
        const ResidualSet base = residuals(dataset, state);
        cv::Mat hessian = cv::Mat::zeros(6, 6, CV_64F);
        cv::Mat gradient = cv::Mat::zeros(6, 1, CV_64F);
        for (int sampleIndex = 0; sampleIndex < dataset.samples.size(); ++sampleIndex) {
            cv::Mat jacobian(6, 6, CV_64F, cv::Scalar(0));
            const double values[6] = {base.rotation.at(sampleIndex)[0], base.rotation.at(sampleIndex)[1],
                                      base.rotation.at(sampleIndex)[2], base.translation.at(sampleIndex)[0],
                                      base.translation.at(sampleIndex)[1], base.translation.at(sampleIndex)[2]};
            for (int parameter = 0; parameter < 6; ++parameter) {
                State perturbed = state;
                const double step = parameter < 3 ? 1e-6 : 1e-7;
                Vector3 rotation = matrix::toRodrigues(perturbed.pose.get_minor<3, 3>(0, 0));
                Vector3 translation{perturbed.pose(0, 3), perturbed.pose(1, 3), perturbed.pose(2, 3)};
                if (parameter < 3) rotation[parameter] += step;
                else translation[parameter - 3] += step;
                perturbed.pose = matrix::fromRodrigues(rotation, translation);
                const ResidualSet changed = residuals(dataset, perturbed);
                const double changedValues[6] = {changed.rotation.at(sampleIndex)[0], changed.rotation.at(sampleIndex)[1],
                                                 changed.rotation.at(sampleIndex)[2], changed.translation.at(sampleIndex)[0],
                                                 changed.translation.at(sampleIndex)[1], changed.translation.at(sampleIndex)[2]};
                for (int component = 0; component < 6; ++component)
                    jacobian.at<double>(component, parameter) = (changedValues[component] - values[component]) / step;
            }
            cv::Mat residual(6, 1, CV_64F);
            for (int component = 0; component < 6; ++component) residual.at<double>(component, 0) = values[component];
            for (int component = 0; component < 6; ++component) {
                const double threshold = component < 3 ? CV_PI / 180.0 : 0.001;
                const double weight = huberWeight(values[component], threshold);
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
            state = trial;
            currentLoss = trialLoss;
            lambda = std::max(lambda * 0.5, 1e-9);
            result.optimizationReport.iterations = iteration + 1;
            if (cv::norm(delta) < 1e-8) {
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
    result.optimizationReport.afterRotationRmseDeg = after.rotationRmseDeg;
    result.optimizationReport.afterTranslationRmseM = after.translationRmseM;
    result.optimizationReport.success = after.success;
    result.optimizationReport.huberOutlierCount = after.outlierCount;
    result.message = result.optimizationReport.converged
                         ? QStringLiteral("非线性精修收敛。")
                         : QStringLiteral("非线性精修完成，但未达到严格收敛条件。");
    return result;
}

} // namespace handeye
