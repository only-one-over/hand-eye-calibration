#pragma once

#include "domain/calibration_types.h"

namespace handeye {

class CalibrationService
{
public:
    static CalibrationResult calibrate(const CalibrationDataset &dataset, CalibrationMethod method);
    static QVector<CalibrationResult> calibrateAll(const CalibrationDataset &dataset);
};

} // namespace handeye
