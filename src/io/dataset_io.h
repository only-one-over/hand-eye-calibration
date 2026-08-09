#pragma once

#include "domain/calibration_types.h"

namespace handeye {

struct IoResult {
    bool success = false;
    QString error;
};

IoResult writeCsv(const QString &filePath, const CalibrationDataset &dataset);
IoResult readCsv(const QString &filePath, CalibrationDataset *dataset,
                 const PoseInputSpec &inputSpec);
IoResult writeJson(const QString &filePath, const CalibrationDataset &dataset);
IoResult readJson(const QString &filePath, CalibrationDataset *dataset);
IoResult writeYaml(const QString &filePath, const CalibrationDataset &dataset);
IoResult writeResultTxt(const QString &filePath, const CalibrationDataset &dataset,
                        const CalibrationResult &result);
IoResult writeResultCpp(const QString &filePath, const CalibrationDataset &dataset,
                        const CalibrationResult &result);
IoResult writeResultPython(const QString &filePath, const CalibrationDataset &dataset,
                           const CalibrationResult &result);

} // namespace handeye
