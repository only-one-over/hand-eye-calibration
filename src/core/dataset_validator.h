#pragma once

#include "domain/calibration_types.h"

namespace handeye {

struct ValidationReport {
    bool valid = false;
    bool relativeMotionDegenerate = false;
    int relativeMotionCount = 0;
    QStringList errors;
    QStringList warnings;
};

ValidationReport validateDataset(const CalibrationDataset &dataset,
                                 const QVector<PoseSample> *samplesOverride = nullptr);

} // namespace handeye
