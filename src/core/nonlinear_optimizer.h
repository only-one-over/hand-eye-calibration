#pragma once

#include "domain/calibration_types.h"

namespace handeye {

class NonlinearOptimizer
{
public:
    static CalibrationResult refinePose(const CalibrationDataset &dataset,
                                        const CalibrationResult &seed);
};

} // namespace handeye
