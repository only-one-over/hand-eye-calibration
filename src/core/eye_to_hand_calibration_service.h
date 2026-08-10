#pragma once

#include "domain/calibration_types.h"

namespace handeye {

class EyeToHandCalibrationService
{
public:
    static CalibrationResult calibrate(const CalibrationDataset &dataset,
                                       CalibrationMethod method);
    static QVector<CalibrationResult> calibrateAll(const CalibrationDataset &dataset);

    static EyeToHandPoseReport evaluatePose(const CalibrationDataset &dataset,
                                             const Matrix4 &cameraToBase,
                                             const Matrix4 &targetToGripper,
                                             const QVector<PoseSample> &samples = {});
    static EyeToHandPointReport evaluatePoint(const CalibrationDataset &dataset,
                                              const Matrix4 &cameraToBase,
                                              const Vector3 &pointInGripper,
                                              const QVector<PointSample> &samples = {});
};

} // namespace handeye
