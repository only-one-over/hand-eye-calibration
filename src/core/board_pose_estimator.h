#pragma once

#include "domain/calibration_types.h"

namespace handeye {

struct BoardPoseEstimate {
    bool success = false;
    ImageSampleStatus status = ImageSampleStatus::NotProcessed;
    Vector3 targetRotation{};
    Vector3 targetTranslation{};
    int detectedCornerCount = 0;
    double reprojectionRmsePx = 0.0;
    QString message;
};

struct BoardCornerDetection {
    bool success = false;
    ImageSampleStatus status = ImageSampleStatus::NotProcessed;
    int imageWidth = 0;
    int imageHeight = 0;
    QVector<Vector2> corners;
    QString message;
};

class BoardPoseEstimator
{
public:
    static BoardCornerDetection detectChessboard(const QString &imagePath, const BoardSpec &board);
    static BoardPoseEstimate estimateChessboard(const QString &imagePath,
                                                const BoardSpec &board,
                                                const CameraIntrinsics &intrinsics);
};

} // namespace handeye
