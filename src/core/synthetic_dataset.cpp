#include "core/synthetic_dataset.h"

#include "core/matrix_utils.h"

#include <opencv2/core.hpp>

namespace handeye {

CalibrationDataset makeSyntheticDataset(int count)
{
    CalibrationDataset dataset;
    dataset.targetPosesReady = true;
    const Vector3 cameraRotation{0.18, -0.22, 0.12};
    const Vector3 cameraTranslation{0.08, -0.04, 0.16};
    const cv::Matx44d cameraToGripper = matrix::fromRodrigues(cameraRotation, cameraTranslation);
    dataset.hasGroundTruth = true;
    dataset.groundTruthCameraToGripper = matrix::toArray(cameraToGripper);

    for (int i = 0; i < std::max(3, count); ++i) {
        const double phase = static_cast<double>(i + 1);
        const Vector3 gripperRotation{0.12 * std::sin(phase * 0.71),
                                      0.18 * std::cos(phase * 0.53),
                                      0.24 * std::sin(phase * 0.37)};
        const Vector3 gripperTranslation{0.10 * std::sin(phase * 0.41),
                                         0.08 * std::cos(phase * 0.29),
                                         0.30 + 0.06 * std::sin(phase * 0.67)};
        const cv::Matx44d gripperToBase = matrix::fromRodrigues(gripperRotation, gripperTranslation);
        const cv::Matx44d targetToCamera = matrix::inverse(cameraToGripper)
                                           * matrix::inverse(gripperToBase);
        PoseSample sample;
        sample.id = i + 1;
        sample.gripperRotation = gripperRotation;
        sample.gripperTranslation = gripperTranslation;
        sample.targetRotation = matrix::toRodrigues(targetToCamera.get_minor<3, 3>(0, 0));
        sample.targetTranslation = {targetToCamera(0, 3), targetToCamera(1, 3), targetToCamera(2, 3)};
        sample.label = QStringLiteral("Synthetic %1").arg(i + 1);
        dataset.samples.append(sample);
    }
    return dataset;
}

} // namespace handeye
