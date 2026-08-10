#include "core/eye_to_hand_calibration_service.h"

#include "core/matrix_utils.h"
#include "core/normalized_huber.h"
#include "core/pose_quality_service.h"

#include <opencv2/calib3d.hpp>

#include <QElapsedTimer>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace handeye {

namespace {

using Transform = cv::Matx44d;

Transform poseOf(const Vector3 &rotation, const Vector3 &translation)
{
    return matrix::fromRodrigues(rotation, translation);
}

cv::Vec3d pointOf(const Transform &transform, const Vector3 &point)
{
    return {transform(0, 0) * point[0] + transform(0, 1) * point[1]
                + transform(0, 2) * point[2] + transform(0, 3),
            transform(1, 0) * point[0] + transform(1, 1) * point[1]
                + transform(1, 2) * point[2] + transform(1, 3),
            transform(2, 0) * point[0] + transform(2, 1) * point[1]
                + transform(2, 2) * point[2] + transform(2, 3)};
}

Vector3 translationOf(const Transform &transform)
{
    return {transform(0, 3), transform(1, 3), transform(2, 3)};
}

double norm3(const cv::Vec3d &value)
{
    return std::sqrt(value.dot(value));
}

bool validPoseSamples(const QVector<PoseSample> &samples, QStringList *errors)
{
    QSet<int> ids;
    for (int index = 0; index < samples.size(); ++index) {
        const PoseSample &sample = samples.at(index);
        if (sample.id <= 0) errors->append(QStringLiteral("第 %1 组样本 ID 必须为正整数。" ).arg(index + 1));
        if (ids.contains(sample.id)) errors->append(QStringLiteral("样本 ID 重复：%1。" ).arg(sample.id));
        ids.insert(sample.id);
        if (!matrix::isFinite(sample.gripperRotation) || !matrix::isFinite(sample.gripperTranslation)
            || !matrix::isFinite(sample.targetRotation) || !matrix::isFinite(sample.targetTranslation))
            errors->append(QStringLiteral("第 %1 组样本包含非有限位姿。" ).arg(index + 1));
    }
    if (errors->isEmpty() && samples.size() >= 3) {
        int nonZeroMotions = 0;
        QVector<Vector3> axes;
        for (int i = 0; i < samples.size(); ++i) {
            for (int j = i + 1; j < samples.size(); ++j) {
                const Transform first = poseOf(samples.at(i).gripperRotation, samples.at(i).gripperTranslation);
                const Transform second = poseOf(samples.at(j).gripperRotation, samples.at(j).gripperTranslation);
                const Vector3 motion = matrix::toRodrigues((matrix::inverse(second) * first).get_minor<3, 3>(0, 0));
                const double angle = std::sqrt(motion[0] * motion[0] + motion[1] * motion[1] + motion[2] * motion[2]);
                if (angle > 1e-5) {
                    ++nonZeroMotions;
                    axes.append({motion[0] / angle, motion[1] / angle, motion[2] / angle});
                }
            }
        }
        double maxCross = 0.0;
        for (int i = 0; i < axes.size(); ++i)
            for (int j = i + 1; j < axes.size(); ++j) {
                const Vector3 cross{axes.at(i)[1] * axes.at(j)[2] - axes.at(i)[2] * axes.at(j)[1],
                                    axes.at(i)[2] * axes.at(j)[0] - axes.at(i)[0] * axes.at(j)[2],
                                    axes.at(i)[0] * axes.at(j)[1] - axes.at(i)[1] * axes.at(j)[0]};
                maxCross = std::max(maxCross, std::sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]));
            }
        if (nonZeroMotions < 2 || maxCross < 0.05)
            errors->append(QStringLiteral("Eye-To-Hand 相对运动退化：需要至少两组非零且不平行的旋转运动。"));
    }
    return errors->isEmpty();
}

bool validPointSamples(const QVector<PointSample> &samples, QStringList *errors)
{
    QSet<int> ids;
    for (int index = 0; index < samples.size(); ++index) {
        const PointSample &sample = samples.at(index);
        if (sample.id <= 0) errors->append(QStringLiteral("第 %1 组样本 ID 必须为正整数。" ).arg(index + 1));
        if (ids.contains(sample.id)) errors->append(QStringLiteral("样本 ID 重复：%1。" ).arg(sample.id));
        ids.insert(sample.id);
        if (!matrix::isFinite(sample.gripperRotation) || !matrix::isFinite(sample.gripperTranslation)
            || !matrix::isFinite(sample.cameraPoint))
            errors->append(QStringLiteral("第 %1 组样本包含非有限数值。" ).arg(index + 1));
    }
    if (errors->isEmpty() && samples.size() >= 2) {
        double pointRange = 0.0;
        double translationRange = 0.0;
        for (int i = 0; i < samples.size(); ++i)
            for (int j = i + 1; j < samples.size(); ++j) {
                const Vector3 &a = samples.at(i).cameraPoint;
                const Vector3 &b = samples.at(j).cameraPoint;
                pointRange = std::max(pointRange, std::sqrt(std::pow(a[0] - b[0], 2.0)
                                                             + std::pow(a[1] - b[1], 2.0)
                                                             + std::pow(a[2] - b[2], 2.0)));
                const Vector3 &ta = samples.at(i).gripperTranslation;
                const Vector3 &tb = samples.at(j).gripperTranslation;
                translationRange = std::max(translationRange, std::sqrt(std::pow(ta[0] - tb[0], 2.0)
                                                                         + std::pow(ta[1] - tb[1], 2.0)
                                                                         + std::pow(ta[2] - tb[2], 2.0)));
            }
        if (pointRange < 1e-6 || translationRange < 1e-6)
            errors->append(QStringLiteral("Eye-To-Hand 点基运动退化：相机点或 TCP 平移没有足够变化。"));
    }
    return errors->isEmpty();
}

EyeToHandPoseReport poseReport(const QVector<PoseSample> &samples, const Transform &cameraToBase,
                               const Transform &targetToGripper)
{
    EyeToHandPoseReport report;
    report.available = true;
    report.cameraToBase = matrix::toArray(cameraToBase);
    report.targetToGripper = matrix::toArray(targetToGripper);
    if (samples.isEmpty()) {
        report.errors << QStringLiteral("没有可评价的 PosePairs 样本。");
        return report;
    }

    double rotationSquared = 0.0;
    double translationSquared = 0.0;
    double rotationSum = 0.0;
    double translationSum = 0.0;
    for (const PoseSample &sample : samples) {
        const Transform robot = poseOf(sample.gripperRotation, sample.gripperTranslation);
        const Transform target = poseOf(sample.targetRotation, sample.targetTranslation);
        const Transform robotTarget = robot * targetToGripper;
        const Transform cameraTarget = cameraToBase * target;
        const Transform error = matrix::inverse(robotTarget) * cameraTarget;
        const double rotation = matrix::rotationAngleDeg(error.get_minor<3, 3>(0, 0));
        const double translation = norm3({error(0, 3), error(1, 3), error(2, 3)});
        EyeToHandPoseResidual residual;
        residual.sampleId = sample.id;
        residual.rotationErrorDeg = rotation;
        residual.translationErrorM = translation;
        residual.outlier = std::sqrt(std::pow(rotation / 1.0, 2.0)
                                     + std::pow(translation / 0.001, 2.0)) > 3.0;
        report.samples.append(residual);
        report.outlierCount += residual.outlier ? 1 : 0;
        rotationSquared += rotation * rotation;
        translationSquared += translation * translation;
        rotationSum += rotation;
        translationSum += translation;
        report.rotationMaxDeg = std::max(report.rotationMaxDeg, rotation);
        report.translationMaxM = std::max(report.translationMaxM, translation);
    }
    const double count = static_cast<double>(samples.size());
    report.rotationRmseDeg = std::sqrt(rotationSquared / count);
    report.translationRmseM = std::sqrt(translationSquared / count);
    report.rotationMeanDeg = rotationSum / count;
    report.translationMeanM = translationSum / count;
    report.success = true;
    if (report.outlierCount > 0)
        report.warnings << QStringLiteral("有 %1 组 Eye-To-Hand 位姿一致性残差超过归一化阈值。")
                              .arg(report.outlierCount);
    return report;
}

EyeToHandPointReport pointReport(const QVector<PointSample> &samples, const Transform &cameraToBase,
                                 const cv::Vec3d &pointInGripper)
{
    EyeToHandPointReport report;
    report.available = true;
    report.cameraToBase = matrix::toArray(cameraToBase);
    report.pointInGripper = {pointInGripper[0], pointInGripper[1], pointInGripper[2]};
    if (samples.isEmpty()) {
        report.errors << QStringLiteral("没有可评价的 FixedPoint3D 样本。");
        return report;
    }

    double squared = 0.0;
    double sum = 0.0;
    for (const PointSample &sample : samples) {
        const Transform robot = poseOf(sample.gripperRotation, sample.gripperTranslation);
        const cv::Vec3d robotPoint = pointOf(robot, report.pointInGripper);
        const cv::Vec3d cameraPoint = pointOf(cameraToBase, sample.cameraPoint);
        const double error = norm3(robotPoint - cameraPoint);
        FixedPointSample residual;
        residual.sampleId = sample.id;
        residual.predictedBasePoint = {cameraPoint[0], cameraPoint[1], cameraPoint[2]};
        residual.residualM = error;
        residual.outlier = error > 0.003;
        report.samples.append(residual);
        squared += error * error;
        sum += error;
        report.maxErrorM = std::max(report.maxErrorM, error);
        report.outlierCount += residual.outlier ? 1 : 0;
    }
    const double count = static_cast<double>(samples.size());
    report.rmseM = std::sqrt(squared / count);
    report.meanErrorM = sum / count;
    report.success = true;
    return report;
}

struct PoseState {
    Transform cameraToBase = Transform::eye();
    Transform targetToGripper = Transform::eye();
};

double poseLoss(const QVector<PoseSample> &samples, const PoseState &state,
                const NormalizedHuberConfig &config)
{
    double loss = 0.0;
    for (const PoseSample &sample : samples) {
        const Transform robot = poseOf(sample.gripperRotation, sample.gripperTranslation);
        const Transform target = poseOf(sample.targetRotation, sample.targetTranslation);
        const Transform error = matrix::inverse(robot * state.targetToGripper)
                                * (state.cameraToBase * target);
        const Vector3 r = matrix::toRodrigues(error.get_minor<3, 3>(0, 0));
        const Vector3 t = translationOf(error);
        loss += NormalizedHuber::evaluate(r, t, config).loss;
    }
    return loss;
}

PoseState refinePose(const QVector<PoseSample> &samples, PoseState state,
                     NonlinearOptimizationReport *report)
{
    const NormalizedHuberConfig config;
    report->available = true;
    const EyeToHandPoseReport before = poseReport(samples, state.cameraToBase, state.targetToGripper);
    report->beforeRotationRmseDeg = before.rotationRmseDeg;
    report->beforeTranslationRmseM = before.translationRmseM;
    double currentLoss = poseLoss(samples, state, config);
    report->normalizedHuberLossBefore = currentLoss;
    double lambda = 1e-3;
    for (int iteration = 0; iteration < 50; ++iteration) {
        cv::Mat hessian = cv::Mat::zeros(12, 12, CV_64F);
        cv::Mat gradient = cv::Mat::zeros(12, 1, CV_64F);
        for (int sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex) {
            const PoseSample &sample = samples.at(sampleIndex);
            const Transform robot = poseOf(sample.gripperRotation, sample.gripperTranslation);
            const Transform target = poseOf(sample.targetRotation, sample.targetTranslation);
            const Transform error = matrix::inverse(robot * state.targetToGripper)
                                    * (state.cameraToBase * target);
            const Vector3 residualRotation = matrix::toRodrigues(error.get_minor<3, 3>(0, 0));
            const Vector3 residualTranslation = translationOf(error);
            const NormalizedHuberEvaluation evaluation = NormalizedHuber::evaluate(
                residualRotation, residualTranslation, config);
            cv::Mat jacobian = cv::Mat::zeros(6, 12, CV_64F);
            const double steps[12] = {1e-6, 1e-6, 1e-6, 1e-7, 1e-7, 1e-7,
                                      1e-6, 1e-6, 1e-6, 1e-7, 1e-7, 1e-7};
            for (int parameter = 0; parameter < 12; ++parameter) {
                PoseState perturbed = state;
                if (parameter < 6) {
                    Vector3 r = matrix::toRodrigues(perturbed.cameraToBase.get_minor<3, 3>(0, 0));
                    if (parameter < 3) r[parameter] += steps[parameter];
                    else perturbed.cameraToBase(parameter - 3, 3) += steps[parameter];
                    perturbed.cameraToBase = poseOf(r, translationOf(perturbed.cameraToBase));
                } else {
                    const int offset = parameter - 6;
                    Vector3 r = matrix::toRodrigues(perturbed.targetToGripper.get_minor<3, 3>(0, 0));
                    if (offset < 3) r[offset] += steps[parameter];
                    else perturbed.targetToGripper(offset - 3, 3) += steps[parameter];
                    perturbed.targetToGripper = poseOf(r, translationOf(perturbed.targetToGripper));
                }
                const Transform changed = matrix::inverse(robot * perturbed.targetToGripper)
                                           * (perturbed.cameraToBase * target);
                const Vector3 changedR = matrix::toRodrigues(changed.get_minor<3, 3>(0, 0));
                const Vector3 changedT = translationOf(changed);
                for (int row = 0; row < 3; ++row) {
                    jacobian.at<double>(row, parameter) = (changedR[row] - residualRotation[row])
                                                          / steps[parameter] / config.rotationScaleRad;
                    jacobian.at<double>(row + 3, parameter) = (changedT[row] - residualTranslation[row])
                                                              / steps[parameter] / config.translationScaleM;
                }
            }
            cv::Mat residual(6, 1, CV_64F);
            for (int row = 0; row < 3; ++row) {
                residual.at<double>(row, 0) = residualRotation[row] / config.rotationScaleRad;
                residual.at<double>(row + 3, 0) = residualTranslation[row] / config.translationScaleM;
            }
            hessian += evaluation.weight * jacobian.t() * jacobian;
            gradient += evaluation.weight * jacobian.t() * residual;
        }
        for (int d = 0; d < 12; ++d)
            hessian.at<double>(d, d) += lambda * (hessian.at<double>(d, d) + 1e-9);
        cv::Mat delta;
        if (!cv::solve(hessian, -gradient, delta, cv::DECOMP_SVD)) break;
        PoseState trial = state;
        Vector3 cameraRotation = matrix::toRodrigues(trial.cameraToBase.get_minor<3, 3>(0, 0));
        Vector3 targetRotation = matrix::toRodrigues(trial.targetToGripper.get_minor<3, 3>(0, 0));
        for (int i = 0; i < 3; ++i) {
            cameraRotation[i] += delta.at<double>(i);
            targetRotation[i] += delta.at<double>(i + 6);
        }
        Vector3 cameraTranslation = translationOf(trial.cameraToBase);
        Vector3 targetTranslation = translationOf(trial.targetToGripper);
        for (int i = 0; i < 3; ++i) {
            cameraTranslation[i] += delta.at<double>(i + 3);
            targetTranslation[i] += delta.at<double>(i + 9);
        }
        trial.cameraToBase = poseOf(cameraRotation, cameraTranslation);
        trial.targetToGripper = poseOf(targetRotation, targetTranslation);
        const double trialLoss = poseLoss(samples, trial, config);
        if (trialLoss < currentLoss) {
            state = trial;
            currentLoss = trialLoss;
            lambda = std::max(lambda * 0.5, 1e-9);
            report->iterations = iteration + 1;
            if (cv::norm(delta) < 1e-7) {
                report->converged = true;
                break;
            }
        } else {
            lambda = std::min(lambda * 4.0, 1e8);
        }
    }
    const EyeToHandPoseReport after = poseReport(samples, state.cameraToBase, state.targetToGripper);
    report->afterRotationRmseDeg = after.rotationRmseDeg;
    report->afterTranslationRmseM = after.translationRmseM;
    report->normalizedHuberLossAfter = currentLoss;
    for (const EyeToHandPoseResidual &sample : after.samples)
        report->huberOutlierCount += std::sqrt(std::pow(sample.rotationErrorDeg, 2.0)
                                               + std::pow(sample.translationErrorM / 0.001, 2.0)) > 1.0;
    report->success = after.success;
    report->message = report->converged ? QStringLiteral("Eye-To-Hand 非线性精修收敛。")
                                       : QStringLiteral("Eye-To-Hand 非线性精修达到停止条件。");
    return state;
}

struct PointState {
    Transform cameraToBase = Transform::eye();
    cv::Vec3d pointInGripper{0.0, 0.0, 0.0};
};

PointState pointLinearInitial(const QVector<PointSample> &samples)
{
    cv::Mat lhs(samples.size() * 3, 15, CV_64F, cv::Scalar(0));
    cv::Mat rhs(samples.size() * 3, 1, CV_64F, cv::Scalar(0));
    for (int i = 0; i < samples.size(); ++i) {
        const PointSample &sample = samples.at(i);
        const Transform robot = poseOf(sample.gripperRotation, sample.gripperTranslation);
        const cv::Matx33d r = robot.get_minor<3, 3>(0, 0);
        const int base = i * 3;
        for (int output = 0; output < 3; ++output) {
            for (int k = 0; k < 3; ++k)
                lhs.at<double>(base + output, output * 3 + k) = sample.cameraPoint[k];
            for (int c = 0; c < 3; ++c) {
                lhs.at<double>(base + output, 9 + c) = (c == output ? 1.0 : 0.0);
                lhs.at<double>(base + output, 12 + c) = -r(output, c);
            }
            rhs.at<double>(base + output, 0) = robot(output, 3);
        }
    }
    // The first nine variables are R_cameraToBase in row-major order.
    cv::Mat solution;
    cv::solve(lhs, rhs, solution, cv::DECOMP_SVD);
    cv::Matx33d raw;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) raw(row, col) = solution.at<double>(row * 3 + col);
    cv::Mat u, s, vt;
    cv::SVD::compute(cv::Mat(raw), s, u, vt);
    cv::Mat projected = u * vt;
    if (cv::determinant(projected) < 0.0) {
        for (int row = 0; row < 3; ++row) u.at<double>(row, 2) *= -1.0;
        projected = u * vt;
    }
    cv::Matx33d rotation;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) rotation(row, col) = projected.at<double>(row, col);
    PointState state;
    state.cameraToBase = poseOf(matrix::toRodrigues(rotation),
                                {solution.at<double>(9), solution.at<double>(10), solution.at<double>(11)});
    state.pointInGripper = {solution.at<double>(12), solution.at<double>(13), solution.at<double>(14)};
    return state;
}

double pointLoss(const QVector<PointSample> &samples, const PointState &state)
{
    double loss = 0.0;
    for (const PointSample &sample : samples) {
        const Transform robot = poseOf(sample.gripperRotation, sample.gripperTranslation);
        const cv::Vec3d residual = pointOf(state.cameraToBase, sample.cameraPoint)
                                   - pointOf(robot, {state.pointInGripper[0], state.pointInGripper[1], state.pointInGripper[2]});
        loss += NormalizedHuber::evaluateTranslation({residual[0], residual[1], residual[2]}).loss;
    }
    return loss;
}

PointState refinePoint(const QVector<PointSample> &samples, PointState state,
                       NonlinearOptimizationReport *report)
{
    const EyeToHandPointReport before = pointReport(samples, state.cameraToBase, state.pointInGripper);
    report->available = true;
    report->beforeTranslationRmseM = before.rmseM;
    double currentLoss = pointLoss(samples, state);
    report->normalizedHuberLossBefore = currentLoss;
    double lambda = 1e-3;
    for (int iteration = 0; iteration < 50; ++iteration) {
        cv::Mat hessian = cv::Mat::zeros(9, 9, CV_64F);
        cv::Mat gradient = cv::Mat::zeros(9, 1, CV_64F);
        for (const PointSample &sample : samples) {
            const Transform robot = poseOf(sample.gripperRotation, sample.gripperTranslation);
            const cv::Vec3d current = pointOf(state.cameraToBase, sample.cameraPoint)
                                      - pointOf(robot, {state.pointInGripper[0], state.pointInGripper[1], state.pointInGripper[2]});
            const NormalizedHuberEvaluation evaluation = NormalizedHuber::evaluateTranslation(
                {current[0], current[1], current[2]});
            cv::Mat jacobian = cv::Mat::zeros(3, 9, CV_64F);
            for (int p = 0; p < 9; ++p) {
                PointState perturbed = state;
                const double step = p < 3 ? 1e-6 : 1e-7;
                if (p < 3) {
                    Vector3 r = matrix::toRodrigues(perturbed.cameraToBase.get_minor<3, 3>(0, 0));
                    r[p] += step;
                    perturbed.cameraToBase = poseOf(r, translationOf(perturbed.cameraToBase));
                } else if (p < 6) {
                    perturbed.cameraToBase(p - 3, 3) += step;
                } else {
                    perturbed.pointInGripper[p - 6] += step;
                }
                const cv::Vec3d changed = pointOf(perturbed.cameraToBase, sample.cameraPoint)
                                           - pointOf(robot, {perturbed.pointInGripper[0], perturbed.pointInGripper[1], perturbed.pointInGripper[2]});
                for (int row = 0; row < 3; ++row)
                    jacobian.at<double>(row, p) = (changed[row] - current[row]) / step / 0.001;
            }
            cv::Mat residual(3, 1, CV_64F);
            for (int row = 0; row < 3; ++row) residual.at<double>(row, 0) = current[row] / 0.001;
            hessian += evaluation.weight * jacobian.t() * jacobian;
            gradient += evaluation.weight * jacobian.t() * residual;
        }
        for (int d = 0; d < 9; ++d)
            hessian.at<double>(d, d) += lambda * (hessian.at<double>(d, d) + 1e-9);
        cv::Mat delta;
        if (!cv::solve(hessian, -gradient, delta, cv::DECOMP_SVD)) break;
        PointState trial = state;
        Vector3 r = matrix::toRodrigues(trial.cameraToBase.get_minor<3, 3>(0, 0));
        for (int i = 0; i < 3; ++i) r[i] += delta.at<double>(i);
        Vector3 t = translationOf(trial.cameraToBase);
        for (int i = 0; i < 3; ++i) t[i] += delta.at<double>(i + 3);
        trial.cameraToBase = poseOf(r, t);
        for (int i = 0; i < 3; ++i) trial.pointInGripper[i] += delta.at<double>(i + 6);
        const double trialLoss = pointLoss(samples, trial);
        if (trialLoss < currentLoss) {
            state = trial;
            currentLoss = trialLoss;
            lambda = std::max(lambda * 0.5, 1e-9);
            report->iterations = iteration + 1;
            if (cv::norm(delta) < 1e-7) {
                report->converged = true;
                break;
            }
        } else {
            lambda = std::min(lambda * 4.0, 1e8);
        }
    }
    const EyeToHandPointReport after = pointReport(samples, state.cameraToBase, state.pointInGripper);
    report->afterTranslationRmseM = after.rmseM;
    report->normalizedHuberLossAfter = currentLoss;
    report->success = after.success;
    report->message = report->converged ? QStringLiteral("Eye-To-Hand 点基非线性精修收敛。")
                                       : QStringLiteral("Eye-To-Hand 点基优化达到停止条件。");
    return state;
}

CalibrationResult solvePose(const CalibrationDataset &dataset, CalibrationMethod method)
{
    CalibrationResult result;
    result.method = method;
    result.seedMethod = method;
    QElapsedTimer timer;
    timer.start();
    const QVector<PoseSample> &samples = dataset.samples;
    if (samples.size() < 5) {
        result.message = QStringLiteral("Eye-To-Hand PosePairs 至少需要 5 组样本。");
        return result;
    }
    QStringList errors;
    if (!validPoseSamples(samples, &errors)) {
        result.message = errors.join(' ');
        result.eyeToHandPoseReport.errors = errors;
        return result;
    }
    try {
        std::vector<cv::Mat> world2cam;
        std::vector<cv::Mat> base2gripper;
        for (const PoseSample &sample : samples) {
            const Transform a = poseOf(sample.gripperRotation, sample.gripperTranslation);
            const Transform b = poseOf(sample.targetRotation, sample.targetTranslation);
            world2cam.emplace_back(cv::Mat(a.get_minor<3, 3>(0, 0)).clone());
            base2gripper.emplace_back(cv::Mat(b.get_minor<3, 3>(0, 0)).clone());
        }
        cv::Matx33d xRotation, yRotation;
        cv::Matx31d xTranslation, yTranslation;
        const auto methodId = method == CalibrationMethod::RobotWorldLi
                                  ? cv::CALIB_ROBOT_WORLD_HAND_EYE_LI
                                  : cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH;
        std::vector<cv::Mat> worldTranslations;
        std::vector<cv::Mat> baseTranslations;
        for (const PoseSample &sample : samples) {
            const Transform a = poseOf(sample.gripperRotation, sample.gripperTranslation);
            const Transform b = poseOf(sample.targetRotation, sample.targetTranslation);
            worldTranslations.emplace_back((cv::Mat_<double>(3, 1) << a(0, 3), a(1, 3), a(2, 3)));
            baseTranslations.emplace_back((cv::Mat_<double>(3, 1) << b(0, 3), b(1, 3), b(2, 3)));
        }
        cv::calibrateRobotWorldHandEye(world2cam, worldTranslations, base2gripper, baseTranslations,
                                       xRotation, xTranslation, yRotation, yTranslation, methodId);
        PoseState state;
        state.targetToGripper = poseOf(matrix::toRodrigues(xRotation),
                                      {xTranslation(0), xTranslation(1), xTranslation(2)});
        state.cameraToBase = poseOf(matrix::toRodrigues(yRotation),
                                    {yTranslation(0), yTranslation(1), yTranslation(2)});
        if (method == CalibrationMethod::Nonlinear) {
            // A nonlinear result starts from Shah and retains the requested method label.
            CalibrationResult seed = solvePose(dataset, CalibrationMethod::RobotWorldShah);
            if (!seed.success) return seed;
            state.cameraToBase = matrix::toMat(seed.cameraToBase);
            state.targetToGripper = matrix::toMat(seed.targetToGripper);
            result.seedMethod = seed.method;
        }
        if (method == CalibrationMethod::Nonlinear) {
            result.optimizationReport = {};
            state = refinePose(samples, state, &result.optimizationReport);
        }
        result.cameraToBase = matrix::toArray(state.cameraToBase);
        result.targetToGripper = matrix::toArray(state.targetToGripper);
        result.eyeToHandPoseReport = poseReport(samples, state.cameraToBase, state.targetToGripper);
        result.axXbReport.available = true;
        result.axXbReport.valid = result.eyeToHandPoseReport.success;
        result.axXbReport.sampleCount = samples.size();
        result.axXbReport.rotationRmseDeg = result.eyeToHandPoseReport.rotationRmseDeg;
        result.axXbReport.translationRmseM = result.eyeToHandPoseReport.translationRmseM;
        result.axXbReport.rotationMeanDeg = result.eyeToHandPoseReport.rotationMeanDeg;
        result.axXbReport.translationMeanM = result.eyeToHandPoseReport.translationMeanM;
        result.axXbReport.rotationMaxDeg = result.eyeToHandPoseReport.rotationMaxDeg;
        result.axXbReport.translationMaxM = result.eyeToHandPoseReport.translationMaxM;
        result.axXbReport.outlierCount = result.eyeToHandPoseReport.outlierCount;
        for (const EyeToHandPoseResidual &sample : result.eyeToHandPoseReport.samples)
            result.axXbReport.sampleResiduals.append({sample.sampleId, sample.rotationErrorDeg,
                                                      sample.translationErrorM,
                                                      std::sqrt(std::pow(sample.rotationErrorDeg / 1.0, 2.0)
                                                                + std::pow(sample.translationErrorM / 0.001, 2.0)),
                                                      0, sample.outlier});
        result.axXbReport.passed = result.eyeToHandPoseReport.rotationRmseDeg <= dataset.passRotationRmseDeg
                                   && result.eyeToHandPoseReport.translationRmseM <= dataset.passTranslationRmseM;
        result.trainingReport = result.axXbReport;
        result.qualityReport = PoseQualityService::evaluatePoseQuality(dataset);
        result.rotationErrorDeg = result.eyeToHandPoseReport.rotationRmseDeg;
        result.translationError = result.eyeToHandPoseReport.translationRmseM;
        result.success = true;
        result.message = result.axXbReport.passed ? QStringLiteral("Eye-To-Hand 计算成功，AX=YB 一致性通过")
                                                    : QStringLiteral("Eye-To-Hand 计算成功，但一致性未通过");
    } catch (const cv::Exception &error) {
        result.message = QStringLiteral("OpenCV Eye-To-Hand 错误：%1").arg(QString::fromStdString(error.what()));
    } catch (const std::exception &error) {
        result.message = QStringLiteral("Eye-To-Hand 计算错误：%1").arg(QString::fromStdString(error.what()));
    }
    result.elapsedMs = timer.elapsed();
    return result;
}

CalibrationResult solvePoint(const CalibrationDataset &dataset)
{
    CalibrationResult result;
    result.method = CalibrationMethod::PointBased;
    result.seedMethod = CalibrationMethod::PointBased;
    QElapsedTimer timer;
    timer.start();
    const QVector<PointSample> &samples = dataset.pointSamples;
    if (samples.size() < 5) {
        result.message = QStringLiteral("Eye-To-Hand FixedPoint3D 至少需要 5 组样本。");
        return result;
    }
    QStringList errors;
    if (!validPointSamples(samples, &errors)) {
        result.message = errors.join(' ');
        result.eyeToHandPointReport.errors = errors;
        return result;
    }
    PointState state = pointLinearInitial(samples);
    result.optimizationReport = {};
    state = refinePoint(samples, state, &result.optimizationReport);
    result.cameraToBase = matrix::toArray(state.cameraToBase);
    result.pointInGripper = {state.pointInGripper[0], state.pointInGripper[1], state.pointInGripper[2]};
    result.eyeToHandPointReport = pointReport(samples, state.cameraToBase, state.pointInGripper);
    result.fixedPointReport.available = result.eyeToHandPointReport.available;
    result.fixedPointReport.success = result.eyeToHandPointReport.success;
    result.fixedPointReport.robustMeanPoint = result.eyeToHandPointReport.pointInGripper;
    result.fixedPointReport.rmseM = result.eyeToHandPointReport.rmseM;
    result.fixedPointReport.meanErrorM = result.eyeToHandPointReport.meanErrorM;
    result.fixedPointReport.maxErrorM = result.eyeToHandPointReport.maxErrorM;
    result.fixedPointReport.outlierCount = result.eyeToHandPointReport.outlierCount;
    result.fixedPointReport.samples = result.eyeToHandPointReport.samples;
    result.axXbReport.available = true;
    result.axXbReport.valid = result.eyeToHandPointReport.success;
    result.axXbReport.sampleCount = samples.size();
    result.axXbReport.translationRmseM = result.eyeToHandPointReport.rmseM;
    result.axXbReport.translationMeanM = result.eyeToHandPointReport.meanErrorM;
    result.axXbReport.translationMaxM = result.eyeToHandPointReport.maxErrorM;
    result.axXbReport.passed = result.eyeToHandPointReport.rmseM <= dataset.passTranslationRmseM;
    for (const FixedPointSample &sample : result.eyeToHandPointReport.samples)
        result.axXbReport.sampleResiduals.append({sample.sampleId, 0.0, sample.residualM,
                                                  sample.residualM / std::max(dataset.passTranslationRmseM, 1e-12),
                                                  0, sample.outlier});
    result.trainingReport = result.axXbReport;
    result.qualityReport = PoseQualityService::evaluatePointQuality(dataset);
    result.translationError = result.eyeToHandPointReport.rmseM;
    result.success = result.eyeToHandPointReport.success;
    result.message = result.axXbReport.passed ? QStringLiteral("Eye-To-Hand 点基标定成功，固定点一致性通过")
                                                : QStringLiteral("Eye-To-Hand 点基标定成功，但固定点一致性未通过");
    result.elapsedMs = timer.elapsed();
    return result;
}

} // namespace

CalibrationResult EyeToHandCalibrationService::calibrate(const CalibrationDataset &dataset,
                                                          CalibrationMethod method)
{
    if (dataset.inputMode == CalibrationInputMode::FixedPoint3D) return solvePoint(dataset);
    return solvePose(dataset, method);
}

QVector<CalibrationResult> EyeToHandCalibrationService::calibrateAll(const CalibrationDataset &dataset)
{
    if (dataset.inputMode == CalibrationInputMode::FixedPoint3D)
        return {solvePoint(dataset)};
    QVector<CalibrationResult> results;
    results.append(solvePose(dataset, CalibrationMethod::RobotWorldShah));
    results.append(solvePose(dataset, CalibrationMethod::RobotWorldLi));
    if (results.first().success) {
        CalibrationResult nonlinear = solvePose(dataset, CalibrationMethod::Nonlinear);
        results.append(nonlinear);
    }
    int recommended = -1;
    double best = std::numeric_limits<double>::max();
    for (int index = 0; index < results.size(); ++index) {
        const CalibrationResult &result = results.at(index);
        if (!result.success) continue;
        const double score = std::pow(result.axXbReport.rotationRmseDeg / std::max(dataset.passRotationRmseDeg, 1e-9), 2.0)
                             + std::pow(result.axXbReport.translationRmseM / std::max(dataset.passTranslationRmseM, 1e-12), 2.0);
        if (score < best) {
            best = score;
            recommended = index;
        }
    }
    if (recommended >= 0) results[recommended].recommended = true;
    return results;
}

EyeToHandPoseReport EyeToHandCalibrationService::evaluatePose(const CalibrationDataset &, const Matrix4 &cameraToBase,
                                                              const Matrix4 &targetToGripper,
                                                              const QVector<PoseSample> &samples)
{
    return poseReport(samples, matrix::toMat(cameraToBase), matrix::toMat(targetToGripper));
}

EyeToHandPointReport EyeToHandCalibrationService::evaluatePoint(const CalibrationDataset &, const Matrix4 &cameraToBase,
                                                                const Vector3 &pointInGripper,
                                                                const QVector<PointSample> &samples)
{
    return pointReport(samples, matrix::toMat(cameraToBase), {pointInGripper[0], pointInGripper[1], pointInGripper[2]});
}

} // namespace handeye
