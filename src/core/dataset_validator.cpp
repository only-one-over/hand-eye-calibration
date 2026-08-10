#include "core/dataset_validator.h"

#include "core/matrix_utils.h"

#include <opencv2/calib3d.hpp>

#include <QSet>

#include <algorithm>
#include <cmath>

namespace handeye {

ValidationReport validateDataset(const CalibrationDataset &dataset,
                                 const QVector<PoseSample> *samplesOverride)
{
    ValidationReport report;
    const QVector<PoseSample> &samples = samplesOverride ? *samplesOverride : dataset.samples;
    if (dataset.mode == CalibrationMode::EyeToHand
        && dataset.inputMode == CalibrationInputMode::FixedPoint3D) {
        if (dataset.pointSamples.size() < 5)
            report.errors << QStringLiteral("Eye-To-Hand FixedPoint3D 至少需要 5 组 TCP 与相机 XYZ 样本。");
        QSet<int> pointIds;
        for (int index = 0; index < dataset.pointSamples.size(); ++index) {
            const PointSample &point = dataset.pointSamples.at(index);
            if (pointIds.contains(point.id))
                report.errors << QStringLiteral("点基样本 ID 重复：%1。" ).arg(point.id);
            pointIds.insert(point.id);
            if (!matrix::isFinite(point.gripperRotation) || !matrix::isFinite(point.gripperTranslation)
                || !matrix::isFinite(point.cameraPoint))
                report.errors << QStringLiteral("第 %1 组点基样本包含非有限数值。" ).arg(index + 1);
        }
        report.valid = report.errors.isEmpty();
        return report;
    }
    if (samples.size() < 3)
        report.errors << QStringLiteral("至少需要 3 组有效样本。");
    if (dataset.mode == CalibrationMode::EyeToHand
        && dataset.inputMode == CalibrationInputMode::PosePairs && samples.size() < 5)
        report.errors << QStringLiteral("Eye-To-Hand PosePairs 建议并要求至少 5 组样本。");

    QSet<int> ids;
    for (int i = 0; i < samples.size(); ++i) {
        const PoseSample &sample = samples.at(i);
        if (ids.contains(sample.id))
            report.errors << QStringLiteral("样本 ID 重复：%1。").arg(sample.id);
        ids.insert(sample.id);
        if (!matrix::isFinite(sample.gripperRotation) || !matrix::isFinite(sample.gripperTranslation)
            || !matrix::isFinite(sample.targetRotation) || !matrix::isFinite(sample.targetTranslation)) {
            report.errors << QStringLiteral("第 %1 组样本包含非有限数值。").arg(i + 1);
        }
    }

    if (samples.size() >= 3) {
        QVector<Vector3> axes;
        double maxAngle = 0.0;
        for (int i = 0; i < samples.size(); ++i) {
            for (int j = i + 1; j < samples.size(); ++j) {
                const cv::Matx44d gi = matrix::fromRodrigues(samples[i].gripperRotation,
                                                              samples[i].gripperTranslation);
                const cv::Matx44d gj = matrix::fromRodrigues(samples[j].gripperRotation,
                                                              samples[j].gripperTranslation);
                const cv::Matx44d motion = matrix::inverse(gj) * gi;
                const Vector3 rotation = matrix::toRodrigues(motion.get_minor<3, 3>(0, 0));
                const double angle = std::sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1]
                                               + rotation[2] * rotation[2]);
                if (angle > 1e-5) {
                    ++report.relativeMotionCount;
                    maxAngle = std::max(maxAngle, angle);
                    axes.append({rotation[0] / angle, rotation[1] / angle, rotation[2] / angle});
                }
            }
        }
        if (report.relativeMotionCount < 2 || maxAngle < 1e-3) {
            report.relativeMotionDegenerate = true;
            report.errors << QStringLiteral("相对运动不足：需要至少两组非零旋转运动。");
        } else {
            double maxCross = 0.0;
            for (int i = 0; i < axes.size(); ++i) {
                for (int j = i + 1; j < axes.size(); ++j) {
                    const Vector3 cross{axes[i][1] * axes[j][2] - axes[i][2] * axes[j][1],
                                        axes[i][2] * axes[j][0] - axes[i][0] * axes[j][2],
                                        axes[i][0] * axes[j][1] - axes[i][1] * axes[j][0]};
                    const double magnitude = std::sqrt(cross[0] * cross[0] + cross[1] * cross[1]
                                                       + cross[2] * cross[2]);
                    maxCross = std::max(maxCross, magnitude);
                }
            }
            if (maxCross < 0.05) {
                report.relativeMotionDegenerate = true;
                report.errors << QStringLiteral("相对运动旋转轴近似平行，标定问题退化；请绕多个轴采样。");
            }
            if (maxAngle < 5.0 * CV_PI / 180.0)
                report.warnings << QStringLiteral("最大相对旋转小于 5°，结果对噪声较敏感。");
        }
    }

    report.valid = report.errors.isEmpty();
    return report;
}

} // namespace handeye
