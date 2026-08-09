#pragma once

#include "domain/calibration_types.h"

namespace handeye {

class PointCalibrationService
{
public:
    static CalibrationResult calibrate(const CalibrationDataset &dataset);
};

} // namespace handeye
