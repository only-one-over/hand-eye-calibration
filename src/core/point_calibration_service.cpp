#include "core/point_calibration_service.h"

#include "core/matrix_utils.h"
#include "core/pose_quality_service.h"

#include <opencv2/core.hpp>

#include <QSet>

#include <algorithm>
#include <cmath>

namespace handeye {

namespace {

cv::Vec3d pointFromPose(const cv::Matx44d &pose, const Vector3 &point)
{
    return {pose(0, 0) * point[0] + pose(0, 1) * point[1] + pose(0, 2) * point[2] + pose(0, 3),
            pose(1, 0) * point[0] + pose(1, 1) * point[1] + pose(1, 2) * point[2] + pose(1, 3),
            pose(2, 0) * point[0] + pose(2, 1) * point[1] + pose(2, 2) * point[2] + pose(2, 3)};
}

double norm3(const cv::Vec3d &value)
{
    return std::sqrt(value.dot(value));
}

Vector3 medianPoint(const QVector<cv::Vec3d> &points)
{
    Vector3 result{};
    for (int axis = 0; axis < 3; ++axis) {
        QVector<double> values;
        for (const cv::Vec3d &point : points) values.append(point[axis]);
        std::sort(values.begin(), values.end());
        if (values.isEmpty()) continue;
        const int middle = values.size() / 2;
        result[axis] = values.size() % 2 == 1
                           ? values.at(middle)
                           : 0.5 * (values.at(middle - 1) + values.at(middle));
    }
    return result;
}

bool validateSamples(const QVector<PointSample> &samples, QStringList *errors)
{
    QSet<int> ids;
    for (int index = 0; index < samples.size(); ++index) {
        const PointSample &sample = samples.at(index);
        if (sample.id <= 0) errors->append(QStringLiteral("第 %1 组样本 ID 必须为正整数。").arg(index + 1));
        if (ids.contains(sample.id)) errors->append(QStringLiteral("样本 ID 重复：%1。").arg(sample.id));
        ids.insert(sample.id);
        if (!matrix::isFinite(sample.gripperRotation) || !matrix::isFinite(sample.gripperTranslation)
            || !matrix::isFinite(sample.cameraPoint))
            errors->append(QStringLiteral("第 %1 组样本包含非有限数值。").arg(index + 1));
    }
    return errors->isEmpty();
}

cv::Matx44d linearInitial(const QVector<PointSample> &samples)
{
    const int rowCount = samples.size() * 3;
    cv::Mat lhs(rowCount, 15, CV_64F, cv::Scalar(0));
    cv::Mat rhs(rowCount, 1, CV_64F, cv::Scalar(0));
    for (int sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex) {
        const PointSample &sample = samples.at(sampleIndex);
        const cv::Matx44d gripper = matrix::fromRodrigues(sample.gripperRotation,
                                                           sample.gripperTranslation);
        const cv::Matx33d robotRotation = gripper.get_minor<3, 3>(0, 0);
        const int baseRow = sampleIndex * 3;
        for (int output = 0; output < 3; ++output) {
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c)
                    lhs.at<double>(baseRow + output, r * 3 + c) = robotRotation(output, r)
                                                                    * sample.cameraPoint[c];
                lhs.at<double>(baseRow + output, 9 + r) = robotRotation(output, r);
            }
            lhs.at<double>(baseRow + output, 12 + output) = -1.0;
            rhs.at<double>(baseRow + output, 0) = -gripper(output, 3);
        }
    }

    cv::Mat solution;
    cv::solve(lhs, rhs, solution, cv::DECOMP_SVD);
    cv::Matx33d rawRotation;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) rawRotation(row, col) = solution.at<double>(row * 3 + col);
    cv::Mat u, singular, vt;
    cv::SVD::compute(cv::Mat(rawRotation), singular, u, vt);
    cv::Mat projected = u * vt;
    cv::Matx33d rotation;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) rotation(row, col) = projected.at<double>(row, col);
    if (cv::determinant(cv::Mat(rotation)) < 0.0) {
        for (int row = 0; row < 3; ++row) u.at<double>(row, 2) *= -1.0;
        projected = u * vt;
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col) rotation(row, col) = projected.at<double>(row, col);
    }

    cv::Mat refineLhs(rowCount, 6, CV_64F, cv::Scalar(0));
    cv::Mat refineRhs(rowCount, 1, CV_64F, cv::Scalar(0));
    for (int sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex) {
        const PointSample &sample = samples.at(sampleIndex);
        const cv::Matx44d gripper = matrix::fromRodrigues(sample.gripperRotation,
                                                           sample.gripperTranslation);
        const cv::Matx33d robotRotation = gripper.get_minor<3, 3>(0, 0);
        const cv::Vec3d cameraPoint(sample.cameraPoint[0], sample.cameraPoint[1], sample.cameraPoint[2]);
        const cv::Vec3d transformed = rotation * cameraPoint;
        for (int output = 0; output < 3; ++output) {
            const int row = sampleIndex * 3 + output;
            for (int column = 0; column < 3; ++column) refineLhs.at<double>(row, column) = robotRotation(output, column);
            refineLhs.at<double>(row, 3) = refineLhs.at<double>(row, 4) = refineLhs.at<double>(row, 5) = 0.0;
            refineLhs.at<double>(row, 3 + output) = -1.0;
            refineRhs.at<double>(row, 0) = -gripper(output, 3)
                                            - robotRotation(output, 0) * transformed[0]
                                            - robotRotation(output, 1) * transformed[1]
                                            - robotRotation(output, 2) * transformed[2];
        }
    }
    cv::Mat refineSolution;
    cv::solve(refineLhs, refineRhs, refineSolution, cv::DECOMP_SVD);
    const Vector3 translation{refineSolution.at<double>(0), refineSolution.at<double>(1), refineSolution.at<double>(2)};
    return matrix::fromRodrigues(matrix::toRodrigues(rotation), translation);
}

struct OptimizationState {
    cv::Matx44d pose = cv::Matx44d::eye();
    cv::Vec3d fixedPoint{0.0, 0.0, 0.0};
};

QVector<cv::Vec3d> predictedPoints(const QVector<PointSample> &samples, const OptimizationState &state)
{
    QVector<cv::Vec3d> result;
    for (const PointSample &sample : samples) {
        const cv::Matx44d gripper = matrix::fromRodrigues(sample.gripperRotation,
                                                           sample.gripperTranslation);
        result.append(pointFromPose(gripper * state.pose, sample.cameraPoint));
    }
    return result;
}

double huberLoss(const cv::Vec3d &residual, double threshold)
{
    const double norm = norm3(residual);
    if (norm <= threshold) return 0.5 * norm * norm;
    return threshold * (norm - 0.5 * threshold);
}

double optimize(const QVector<PointSample> &samples, OptimizationState *state,
                NonlinearOptimizationReport *report)
{
    const double threshold = 0.001;
    auto lossFor = [&](const OptimizationState &candidate) {
        const QVector<cv::Vec3d> points = predictedPoints(samples, candidate);
        double loss = 0.0;
        for (const cv::Vec3d &point : points) loss += huberLoss(point - candidate.fixedPoint, threshold);
        return loss;
    };
    double currentLoss = lossFor(*state);
    double lambda = 1e-3;
    for (int iteration = 0; iteration < 50; ++iteration) {
        const QVector<cv::Vec3d> points = predictedPoints(samples, *state);
        cv::Mat hessian = cv::Mat::zeros(9, 9, CV_64F);
        cv::Mat gradient = cv::Mat::zeros(9, 1, CV_64F);
        for (int sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex) {
            const cv::Vec3d residual = points.at(sampleIndex) - state->fixedPoint;
            const double residualNorm = norm3(residual);
            const double weight = residualNorm <= threshold ? 1.0 : threshold / std::max(residualNorm, 1e-12);
            cv::Mat jacobian(3, 9, CV_64F, cv::Scalar(0));
            for (int parameter = 0; parameter < 9; ++parameter) {
                OptimizationState perturbed = *state;
                const double step = parameter < 3 ? 1e-6 : 1e-7;
                if (parameter < 3) {
                    Vector3 rotation = matrix::toRodrigues(perturbed.pose.get_minor<3, 3>(0, 0));
                    rotation[parameter] += step;
                    perturbed.pose = matrix::fromRodrigues(rotation,
                                                           {perturbed.pose(0, 3), perturbed.pose(1, 3), perturbed.pose(2, 3)});
                } else if (parameter < 6) {
                    perturbed.pose(parameter - 3, 3) += step;
                } else {
                    perturbed.fixedPoint[parameter - 6] += step;
                }
                const cv::Vec3d changed = predictedPoints(samples, perturbed).at(sampleIndex)
                                           - perturbed.fixedPoint;
                const cv::Vec3d derivative = (changed - residual) * (1.0 / step);
                for (int row = 0; row < 3; ++row) jacobian.at<double>(row, parameter) = derivative[row];
            }
            cv::Mat residualMat(3, 1, CV_64F);
            for (int row = 0; row < 3; ++row) residualMat.at<double>(row, 0) = residual[row];
            hessian += weight * jacobian.t() * jacobian;
            gradient += weight * jacobian.t() * residualMat;
        }
        for (int diagonal = 0; diagonal < 9; ++diagonal)
            hessian.at<double>(diagonal, diagonal) += lambda * (hessian.at<double>(diagonal, diagonal) + 1e-9);
        cv::Mat delta;
        if (!cv::solve(hessian, -gradient, delta, cv::DECOMP_SVD)) break;
        OptimizationState trial = *state;
        Vector3 rotation = matrix::toRodrigues(trial.pose.get_minor<3, 3>(0, 0));
        rotation[0] += delta.at<double>(0);
        rotation[1] += delta.at<double>(1);
        rotation[2] += delta.at<double>(2);
        trial.pose = matrix::fromRodrigues(rotation,
                                           {trial.pose(0, 3) + delta.at<double>(3),
                                            trial.pose(1, 3) + delta.at<double>(4),
                                            trial.pose(2, 3) + delta.at<double>(5)});
        trial.fixedPoint += cv::Vec3d(delta.at<double>(6), delta.at<double>(7), delta.at<double>(8));
        const double trialLoss = lossFor(trial);
        if (trialLoss < currentLoss) {
            *state = trial;
            currentLoss = trialLoss;
            lambda = std::max(lambda * 0.5, 1e-9);
            report->iterations = iteration + 1;
            if (cv::norm(delta) < 1e-8) {
                report->converged = true;
                break;
            }
        } else {
            lambda = std::min(lambda * 4.0, 1e8);
        }
    }
    return currentLoss;
}

FixedPointReport pointReport(const QVector<PointSample> &samples, const OptimizationState &state)
{
    FixedPointReport report;
    report.available = true;
    report.success = !samples.isEmpty();
    report.robustMeanPoint = {state.fixedPoint[0], state.fixedPoint[1], state.fixedPoint[2]};
    const QVector<cv::Vec3d> points = predictedPoints(samples, state);
    double squared = 0.0;
    double sum = 0.0;
    for (int index = 0; index < points.size(); ++index) {
        const double error = norm3(points.at(index) - state.fixedPoint);
        FixedPointSample sample;
        sample.sampleId = samples.at(index).id;
        sample.predictedBasePoint = {points.at(index)[0], points.at(index)[1], points.at(index)[2]};
        sample.residualM = error;
        sample.outlier = error > 0.003;
        report.samples.append(sample);
        report.outlierCount += sample.outlier ? 1 : 0;
        squared += error * error;
        sum += error;
        report.maxErrorM = std::max(report.maxErrorM, error);
    }
    const double count = static_cast<double>(std::max<qsizetype>(1, points.size()));
    report.rmseM = std::sqrt(squared / count);
    report.meanErrorM = sum / count;
    if (report.outlierCount > 0)
        report.warnings << QStringLiteral("有 %1 个点基样本超过 3 mm 残差阈值。").arg(report.outlierCount);
    return report;
}

} // namespace

CalibrationResult PointCalibrationService::calibrate(const CalibrationDataset &dataset)
{
    CalibrationResult result;
    result.method = CalibrationMethod::PointBased;
    result.qualityReport = PoseQualityService::evaluatePointQuality(dataset);
    if (dataset.inputMode != CalibrationInputMode::FixedPoint3D) {
        result.message = QStringLiteral("当前数据不是 FixedPoint3D 模式。");
        return result;
    }
    if (dataset.pointSamples.size() < 5) {
        result.message = QStringLiteral("点基手眼标定至少需要 5 组样本。");
        return result;
    }
    QStringList errors;
    if (!validateSamples(dataset.pointSamples, &errors)) {
        result.message = errors.join(' ');
        return result;
    }
    try {
        OptimizationState state;
        state.pose = linearInitial(dataset.pointSamples);
        const QVector<cv::Vec3d> initialPoints = predictedPoints(dataset.pointSamples, state);
        state.fixedPoint = cv::Vec3d(medianPoint(initialPoints)[0], medianPoint(initialPoints)[1], medianPoint(initialPoints)[2]);
        NonlinearOptimizationReport optimization;
        optimization.available = true;
        const FixedPointReport before = pointReport(dataset.pointSamples, state);
        optimization.beforeTranslationRmseM = before.rmseM;
        optimize(dataset.pointSamples, &state, &optimization);
        const FixedPointReport after = pointReport(dataset.pointSamples, state);
        optimization.afterTranslationRmseM = after.rmseM;
        optimization.afterRotationRmseDeg = 0.0;
        optimization.success = after.success;
        optimization.message = optimization.converged ? QStringLiteral("点基非线性优化收敛。")
                                                        : QStringLiteral("点基优化达到迭代上限或停止条件。");
        result.cameraToGripper = matrix::toArray(state.pose);
        result.fixedPointReport = after;
        result.optimizationReport = optimization;
        result.trainingReport.available = true;
        result.trainingReport.valid = after.success;
        result.trainingReport.passed = after.rmseM <= dataset.passTranslationRmseM && after.outlierCount == 0;
        result.trainingReport.sampleCount = dataset.pointSamples.size();
        result.trainingReport.translationRmseM = after.rmseM;
        result.trainingReport.translationMeanM = after.meanErrorM;
        result.trainingReport.translationMaxM = after.maxErrorM;
        result.success = after.success;
        result.message = result.trainingReport.passed ? QStringLiteral("点基标定成功，固定点残差通过")
                                                       : QStringLiteral("点基标定成功，但固定点残差未通过");
    } catch (const cv::Exception &error) {
        result.message = QStringLiteral("OpenCV 错误：%1").arg(QString::fromStdString(error.what()));
    } catch (const std::exception &error) {
        result.message = QStringLiteral("点基计算错误：%1").arg(QString::fromStdString(error.what()));
    }
    return result;
}

} // namespace handeye
