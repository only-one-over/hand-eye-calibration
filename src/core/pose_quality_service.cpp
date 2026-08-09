#include "core/pose_quality_service.h"

#include "core/matrix_utils.h"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <QSet>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace handeye {

namespace {

double norm3(const cv::Vec3d &value)
{
    return std::sqrt(value.dot(value));
}

cv::Vec3d translationOf(const cv::Matx44d &pose)
{
    return {pose(0, 3), pose(1, 3), pose(2, 3)};
}

double translationDistance(const cv::Vec3d &left, const cv::Vec3d &right)
{
    return norm3(left - right);
}

double median(QVector<double> values)
{
    if (values.isEmpty()) return 0.0;
    std::sort(values.begin(), values.end());
    const int middle = values.size() / 2;
    if (values.size() % 2 == 1) return values.at(middle);
    return 0.5 * (values.at(middle - 1) + values.at(middle));
}

cv::Matx33d robustRotationMean(const QVector<cv::Matx33d> &rotations)
{
    if (rotations.isEmpty()) return cv::Matx33d::eye();
    cv::Matx33d mean = rotations.first();
    for (int iteration = 0; iteration < 20; ++iteration) {
        cv::Vec3d deltaSum(0.0, 0.0, 0.0);
        double weightSum = 0.0;
        for (const cv::Matx33d &rotation : rotations) {
            const cv::Vec3d delta = [&] {
                const cv::Matx33d relative = mean.t() * rotation;
                const Vector3 vector = matrix::toRodrigues(relative);
                return cv::Vec3d(vector[0], vector[1], vector[2]);
            }();
            const double angle = norm3(delta);
            const double huberScale = 0.08;
            const double weight = angle <= huberScale ? 1.0 : huberScale / std::max(angle, 1e-12);
            deltaSum += weight * delta;
            weightSum += weight;
        }
        if (weightSum <= 1e-12) break;
        const cv::Vec3d update = deltaSum * (1.0 / weightSum);
        if (norm3(update) < 1e-10) break;
        cv::Matx33d updateMatrix;
        cv::Rodrigues(update, updateMatrix);
        mean = mean * updateMatrix;
    }
    return mean;
}

struct AxisSummary {
    double maxAngleDeg = 0.0;
    int rank = 0;
};

AxisSummary relativeAxisSummary(const CalibrationDataset &dataset)
{
    QVector<cv::Vec3d> axes;
    double maxAngle = 0.0;
    const int sampleCount = dataset.inputMode == CalibrationInputMode::FixedPoint3D
                                ? dataset.pointSamples.size()
                                : dataset.samples.size();
    auto poseAt = [&](int index) {
        if (dataset.inputMode == CalibrationInputMode::FixedPoint3D) {
            const PointSample &sample = dataset.pointSamples.at(index);
            return matrix::fromRodrigues(sample.gripperRotation, sample.gripperTranslation);
        }
        const PoseSample &sample = dataset.samples.at(index);
        return matrix::fromRodrigues(sample.gripperRotation, sample.gripperTranslation);
    };
    for (int i = 0; i < sampleCount; ++i) {
        const cv::Matx44d left = poseAt(i);
        for (int j = i + 1; j < sampleCount; ++j) {
            const cv::Matx44d right = poseAt(j);
            const cv::Matx33d rotation = right.get_minor<3, 3>(0, 0).t()
                                         * left.get_minor<3, 3>(0, 0);
            const Vector3 vector = matrix::toRodrigues(rotation);
            const cv::Vec3d motion(vector[0], vector[1], vector[2]);
            const double angle = norm3(motion);
            maxAngle = std::max(maxAngle, angle);
            if (angle > 1e-5) axes.append(motion * (1.0 / angle));
        }
    }

    AxisSummary summary;
    summary.maxAngleDeg = maxAngle * 180.0 / CV_PI;
    if (axes.size() < 2) return summary;
    cv::Mat axisMatrix(static_cast<int>(axes.size()), 3, CV_64F);
    for (int row = 0; row < axes.size(); ++row)
        for (int col = 0; col < 3; ++col) axisMatrix.at<double>(row, col) = axes.at(row)[col];
    cv::Mat singularValues;
    cv::SVD::compute(axisMatrix, singularValues);
    for (int i = 0; i < singularValues.rows; ++i)
        if (singularValues.at<double>(i) > 0.15) ++summary.rank;
    return summary;
}

int sampleScore(int count)
{
    if (count < 5) return 0;
    if (count < 12) return 12;
    if (count < 20) return 20;
    return 25;
}

int rotationScore(double angleDeg)
{
    if (angleDeg <= 5.0) return 0;
    if (angleDeg < 30.0) return static_cast<int>(12.0 + (angleDeg - 5.0) * 8.0 / 25.0);
    if (angleDeg <= 45.0) return 20;
    return 25;
}

int axisScore(int rank)
{
    if (rank < 2) return 0;
    if (rank == 2) return 12;
    return 25;
}

int spatialScoreForPose(const CalibrationDataset &dataset, bool *nearMidFar, bool *fullFov,
                        bool *imageAvailable)
{
    QVector<double> depths;
    QVector<cv::Point2f> imagePoints;
    bool hasImage = false;
    for (const PoseSample &sample : dataset.samples) {
        depths.append(std::abs(sample.targetTranslation[2]));
        if (sample.imageWidth > 0 && sample.imageHeight > 0) {
            hasImage = true;
            imagePoints.emplace_back(static_cast<float>(sample.imageCenterXNorm),
                                     static_cast<float>(sample.imageCenterYNorm));
        }
    }
    if (imageAvailable) *imageAvailable = hasImage;
    std::sort(depths.begin(), depths.end());
    const double depthRange = depths.isEmpty() ? 0.0 : depths.last() - depths.first();
    const bool distanceVaried = depthRange > 0.05 * std::max(depths.isEmpty() ? 1.0 : depths.last(), 1e-6);
    const bool distanceThreeLevel = depths.size() >= 3 && depthRange
                                    > 0.2 * std::max(depths.isEmpty() ? 1.0 : depths.last(), 1e-6);
    if (nearMidFar) *nearMidFar = distanceThreeLevel;

    int score = distanceVaried ? 6 : 0;
    if (distanceThreeLevel) score = 12;
    if (!hasImage) {
        if (dataset.samples.size() >= 4) score += 6;
        if (dataset.samples.size() >= 8 && distanceThreeLevel) score += 7;
        return std::min(score, 25);
    }

    QSet<int> quadrants;
    for (const cv::Point2f &point : imagePoints) {
        const int quadrant = (point.x >= 0.5F ? 1 : 0) + (point.y >= 0.5F ? 2 : 0);
        quadrants.insert(quadrant);
    }
    const bool full = quadrants.size() >= 4;
    if (fullFov) *fullFov = full;
    if (quadrants.size() >= 2) score += 6;
    if (full) score += 7;
    return std::min(score, 25);
}

PoseQualityReport qualityFor(const CalibrationDataset &dataset, bool pointMode)
{
    PoseQualityReport report;
    report.available = true;
    const int sampleCount = pointMode ? dataset.pointSamples.size() : dataset.samples.size();
    report.sampleScore = sampleScore(sampleCount);

    const AxisSummary axes = relativeAxisSummary(dataset);
    report.maxRelativeRotationDeg = axes.maxAngleDeg;
    report.independentAxisCount = axes.rank;
    report.rotationAmplitudeScore = rotationScore(axes.maxAngleDeg);
    report.rotationAxisScore = axisScore(axes.rank);
    if (pointMode) {
        QVector<double> depths;
        for (const PointSample &sample : dataset.pointSamples) depths.append(std::abs(sample.cameraPoint[2]));
        std::sort(depths.begin(), depths.end());
        const double range = depths.isEmpty() ? 0.0 : depths.last() - depths.first();
        report.nearMidFarCoverage = depths.size() >= 3 && range > 0.2 * std::max(depths.last(), 1e-6);
        report.fullFovCoverage = false;
        report.spatialDistributionScore = report.nearMidFarCoverage ? 19 : (depths.size() >= 3 ? 12 : 0);
    } else {
        report.spatialDistributionScore = spatialScoreForPose(dataset, &report.nearMidFarCoverage,
                                                              &report.fullFovCoverage,
                                                              &report.imageCoverageAvailable);
    }
    report.totalScore = report.sampleScore + report.rotationAmplitudeScore
                        + report.rotationAxisScore + report.spatialDistributionScore;
    report.calculable = sampleCount >= 5 && axes.rank >= 2 && axes.maxAngleDeg > 5.0;
    if (!report.calculable) {
        report.level = QStringLiteral("无法计算");
        report.warnings << QStringLiteral("样本不足或相对运动退化，不能保证标定可解。");
    } else if (sampleCount >= 20 && axes.maxAngleDeg > 45.0 && axes.rank >= 3
               && report.nearMidFarCoverage && (pointMode || report.fullFovCoverage)
               && report.totalScore >= 90) {
        report.level = QStringLiteral("高精度");
    } else if (sampleCount >= 12 && axes.maxAngleDeg > 30.0 && axes.rank >= 2
               && report.totalScore >= 70) {
        report.level = QStringLiteral("推荐");
    } else {
        report.level = QStringLiteral("可计算");
    }
    return report;
}

} // namespace

FixedTargetPoseReport PoseQualityService::computeFixedTargetPose(const CalibrationDataset &dataset,
                                                                 const Matrix4 &cameraToGripper,
                                                                 int referenceSampleId)
{
    FixedTargetPoseReport report;
    report.available = dataset.inputMode == CalibrationInputMode::PosePairs;
    report.referenceSampleId = referenceSampleId;
    if (!report.available || dataset.samples.isEmpty()) {
        report.errors << QStringLiteral("当前数据不是 PosePairs 模式或没有样本。");
        return report;
    }

    const cv::Matx44d handEye = matrix::toMat(cameraToGripper);
    QVector<cv::Matx44d> predicted;
    QVector<cv::Matx33d> rotations;
    QVector<cv::Vec3d> translations;
    for (const PoseSample &sample : dataset.samples) {
        const cv::Matx44d gripper = matrix::fromRodrigues(sample.gripperRotation,
                                                           sample.gripperTranslation);
        const cv::Matx44d target = matrix::fromRodrigues(sample.targetRotation,
                                                         sample.targetTranslation);
        const cv::Matx44d pose = gripper * handEye * target;
        predicted.append(pose);
        rotations.append(pose.get_minor<3, 3>(0, 0));
        translations.append(translationOf(pose));
    }

    const cv::Matx33d meanRotation = robustRotationMean(rotations);
    Vector3 meanRotationVector = matrix::toRodrigues(meanRotation);
    QVector<double> xValues;
    QVector<double> yValues;
    QVector<double> zValues;
    for (const cv::Vec3d &item : translations) {
        xValues.append(item[0]);
        yValues.append(item[1]);
        zValues.append(item[2]);
    }
    Vector3 meanTranslation{median(xValues), median(yValues), median(zValues)};
    cv::Matx44d meanPose = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) meanPose(row, col) = meanRotation(row, col);
    meanPose(0, 3) = meanTranslation[0];
    meanPose(1, 3) = meanTranslation[1];
    meanPose(2, 3) = meanTranslation[2];
    report.robustMeanPose = matrix::toArray(meanPose);
    report.robustMeanRotation = meanRotationVector;
    report.robustMeanTranslation = meanTranslation;

    cv::Matx44d referencePose = meanPose;
    if (referenceSampleId >= 0) {
        for (int index = 0; index < dataset.samples.size(); ++index)
            if (dataset.samples.at(index).id == referenceSampleId) referencePose = predicted.at(index);
    }
    double rotationSquared = 0.0;
    double translationSquared = 0.0;
    double rotationSum = 0.0;
    double translationSum = 0.0;
    for (int index = 0; index < predicted.size(); ++index) {
        const cv::Matx44d &pose = predicted.at(index);
        const double rotationError = matrix::rotationAngleDeg(meanRotation.t() * pose.get_minor<3, 3>(0, 0));
        const double translationError = translationDistance(translationOf(pose), cv::Vec3d(meanTranslation[0], meanTranslation[1], meanTranslation[2]));
        const double refRotationError = matrix::rotationAngleDeg(referencePose.get_minor<3, 3>(0, 0).t()
                                                                  * pose.get_minor<3, 3>(0, 0));
        const double refTranslationError = translationDistance(translationOf(pose), translationOf(referencePose));
        FixedTargetPoseSample sample;
        sample.sampleId = dataset.samples.at(index).id;
        sample.predictedPose = matrix::toArray(pose);
        sample.predictedRotation = matrix::toRodrigues(pose.get_minor<3, 3>(0, 0));
        const cv::Vec3d translation = translationOf(pose);
        sample.predictedTranslation = {translation[0], translation[1], translation[2]};
        sample.rotationErrorToMeanDeg = rotationError;
        sample.translationErrorToMeanM = translationError;
        sample.rotationErrorToReferenceDeg = refRotationError;
        sample.translationErrorToReferenceM = refTranslationError;
        sample.outlier = std::sqrt(std::pow(rotationError / 1.0, 2.0)
                                    + std::pow(translationError / 0.001, 2.0)) > 3.0;
        report.outlierCount += sample.outlier ? 1 : 0;
        report.samples.append(sample);
        rotationSquared += rotationError * rotationError;
        translationSquared += translationError * translationError;
        rotationSum += rotationError;
        translationSum += translationError;
        report.rotationMaxDeg = std::max(report.rotationMaxDeg, rotationError);
        report.translationMaxM = std::max(report.translationMaxM, translationError);
    }
    const double count = static_cast<double>(predicted.size());
    report.rotationRmseDeg = std::sqrt(rotationSquared / std::max(count, 1.0));
    report.translationRmseM = std::sqrt(translationSquared / std::max(count, 1.0));
    report.rotationMeanDeg = rotationSum / std::max(count, 1.0);
    report.translationMeanM = translationSum / std::max(count, 1.0);
    report.success = true;
    if (report.outlierCount > 0)
        report.warnings << QStringLiteral("固定 target pose 中有 %1 组样本超过鲁棒残差阈值。")
                               .arg(report.outlierCount);
    return report;
}

PoseQualityReport PoseQualityService::evaluatePoseQuality(const CalibrationDataset &dataset)
{
    return qualityFor(dataset, false);
}

PoseQualityReport PoseQualityService::evaluatePointQuality(const CalibrationDataset &dataset)
{
    return qualityFor(dataset, true);
}

} // namespace handeye
