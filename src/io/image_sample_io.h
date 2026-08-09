#pragma once

#include "domain/calibration_types.h"
#include "io/dataset_io.h"

namespace handeye {

// CSV schema: id,image_path,tx,ty,tz,r1,r2,r3[,r4].
// Robot pose values are normalized according to inputSpec; target pose is
// populated later by BoardPoseEstimator.
IoResult readRobotPoseCsv(const QString &filePath, CalibrationDataset *dataset,
                          const PoseInputSpec &inputSpec);

IoResult readPoseImageCsv(const QString &filePath, CalibrationDataset *dataset,
                          const PoseInputSpec &inputSpec);

} // namespace handeye
