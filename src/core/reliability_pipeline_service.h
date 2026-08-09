#pragma once

#include "domain/calibration_types.h"

namespace handeye {

struct ReliabilityPipelineExecution {
    CalibrationDataset refinedDataset;
    CalibrationResult finalResult;
    ReliabilityPipelineReport report;
};

class ReliabilityPipelineService
{
public:
    static ReliabilityPipelineExecution run(const CalibrationDataset &dataset,
                                             int bootstrapResamples = 200,
                                             double confidenceLevel = 0.95);
};

} // namespace handeye
