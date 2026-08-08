#pragma once

#include "domain/calibration_types.h"

namespace handeye {

struct IoResult {
    bool success = false;
    QString error;
};

IoResult writeCsv(const QString &filePath, const CalibrationDataset &dataset);
IoResult readCsv(const QString &filePath, CalibrationDataset *dataset);
IoResult writeJson(const QString &filePath, const CalibrationDataset &dataset);
IoResult readJson(const QString &filePath, CalibrationDataset *dataset);

} // namespace handeye
