#pragma once

#include "domain/calibration_types.h"

namespace handeye {

class PoseQualityService
{
public:
    static FixedTargetPoseReport computeFixedTargetPose(const CalibrationDataset &dataset,
                                                        const Matrix4 &cameraToGripper,
                                                        int referenceSampleId = -1);
    static PoseQualityReport evaluatePoseQuality(const CalibrationDataset &dataset);
    static PoseQualityReport evaluatePointQuality(const CalibrationDataset &dataset);
};

} // namespace handeye
