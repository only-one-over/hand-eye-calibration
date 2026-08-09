#pragma once

#include "domain/calibration_types.h"

namespace handeye {

struct CameraCalibrationOptions {
    int minValidViews = 6;
    double outlierThresholdPx = 1.0;
};

class CameraCalibrationService
{
public:
    static CameraCalibrationReport detectImages(const QStringList &imagePaths, const BoardSpec &board);
    static CameraCalibrationReport calibrateImages(const QStringList &imagePaths,
                                                   const BoardSpec &board,
                                                   const CameraCalibrationOptions &options = {});
    static CameraCalibrationReport calibrateObservations(const QVector<CameraCalibrationSample> &samples,
                                                         const BoardSpec &board,
                                                         const CameraCalibrationOptions &options = {});
};

} // namespace handeye
