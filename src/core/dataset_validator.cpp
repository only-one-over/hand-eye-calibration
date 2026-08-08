#include "core/dataset_validator.h"

#include "core/matrix_utils.h"

#include <cmath>

namespace handeye {

ValidationReport validateDataset(const CalibrationDataset &dataset)
{
    ValidationReport report;
    if (dataset.samples.size() < 3)
        report.errors << QStringLiteral("至少需要 3 组有效样本。");

    for (int i = 0; i < dataset.samples.size(); ++i) {
        const PoseSample &sample = dataset.samples.at(i);
        if (!matrix::isFinite(sample.gripperRotation) || !matrix::isFinite(sample.gripperTranslation)
            || !matrix::isFinite(sample.targetRotation) || !matrix::isFinite(sample.targetTranslation)) {
            report.errors << QStringLiteral("第 %1 组样本包含非有限数值。").arg(i + 1);
        }
    }

    if (dataset.samples.size() >= 3) {
        double maxRotation = 0.0;
        for (const PoseSample &sample : dataset.samples) {
            const double magnitude = std::sqrt(sample.gripperRotation[0] * sample.gripperRotation[0]
                                               + sample.gripperRotation[1] * sample.gripperRotation[1]
                                               + sample.gripperRotation[2] * sample.gripperRotation[2]);
            maxRotation = std::max(maxRotation, magnitude);
        }
        if (maxRotation < 1e-3)
            report.warnings << QStringLiteral("机器人运动旋转变化很小，结果可能退化；建议绕多个轴采样。");
    }

    report.valid = report.errors.isEmpty();
    return report;
}

} // namespace handeye
