#pragma once

#include "domain/calibration_types.h"

namespace handeye {

struct ValidationReport {
    bool valid = false;
    QStringList errors;
    QStringList warnings;
};

ValidationReport validateDataset(const CalibrationDataset &dataset);

} // namespace handeye
