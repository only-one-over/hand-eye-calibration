#include "core/board_pose_estimator.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

namespace handeye {

namespace {

cv::Mat cameraMatrixToCv(const Matrix3 &matrix)
{
    cv::Mat result(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            result.at<double>(row, col) = matrix[row][col];
    return result;
}

cv::Mat distortionToCv(const Vector5 &coeffs)
{
    cv::Mat result(1, 5, CV_64F);
    for (int index = 0; index < 5; ++index) result.at<double>(0, index) = coeffs[index];
    return result;
}

std::vector<cv::Point2f> toCvCorners(const QVector<Vector2> &corners)
{
    std::vector<cv::Point2f> result;
    result.reserve(static_cast<size_t>(corners.size()));
    for (const Vector2 &corner : corners)
        result.emplace_back(static_cast<float>(corner[0]), static_cast<float>(corner[1]));
    return result;
}

} // namespace

BoardCornerDetection BoardPoseEstimator::detectChessboard(const QString &imagePath, const BoardSpec &board)
{
    BoardCornerDetection result;
    if (imagePath.trimmed().isEmpty()) {
        result.status = ImageSampleStatus::ImageMissing;
        result.message = QStringLiteral("Image path is empty.");
        return result;
    }
    if (board.innerCornersX < 2 || board.innerCornersY < 2 || board.squareSizeM <= 0.0) {
        result.status = ImageSampleStatus::DetectionFailed;
        result.message = QStringLiteral("Chessboard dimensions and square size must be positive.");
        return result;
    }

    const cv::Mat image = cv::imread(imagePath.toLocal8Bit().constData(), cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        result.status = ImageSampleStatus::ImageMissing;
        result.message = QStringLiteral("Cannot load image: %1").arg(imagePath);
        return result;
    }
    result.imageWidth = image.cols;
    result.imageHeight = image.rows;

    const cv::Size patternSize(board.innerCornersX, board.innerCornersY);
    std::vector<cv::Point2f> corners;
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    if (!cv::findChessboardCorners(image, patternSize, corners, flags)) {
        result.status = ImageSampleStatus::DetectionFailed;
        result.message = QStringLiteral("Chessboard corners were not detected.");
        return result;
    }

    cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    result.corners.reserve(static_cast<int>(corners.size()));
    for (const cv::Point2f &corner : corners)
        result.corners.append({corner.x, corner.y});
    result.success = true;
    result.status = ImageSampleStatus::PoseEstimated;
    result.message = QStringLiteral("Chessboard corners detected.");
    return result;
}

BoardPoseEstimate BoardPoseEstimator::estimateChessboard(const QString &imagePath,
                                                         const BoardSpec &board,
                                                         const CameraIntrinsics &intrinsics)
{
    BoardPoseEstimate result;
    if (!intrinsics.valid) {
        result.status = ImageSampleStatus::PnpFailed;
        result.message = QStringLiteral("Valid camera intrinsics are required for solvePnP.");
        return result;
    }

    const BoardCornerDetection detection = detectChessboard(imagePath, board);
    if (!detection.success) {
        result.status = detection.status;
        result.message = detection.message;
        return result;
    }
    const std::vector<cv::Point2f> corners = toCvCorners(detection.corners);

    std::vector<cv::Point3f> objectPoints;
    objectPoints.reserve(static_cast<size_t>(board.innerCornersX * board.innerCornersY));
    for (int row = 0; row < board.innerCornersY; ++row)
        for (int col = 0; col < board.innerCornersX; ++col)
            objectPoints.emplace_back(static_cast<float>(col * board.squareSizeM),
                                      static_cast<float>(row * board.squareSizeM), 0.0F);

    cv::Mat rvec;
    cv::Mat tvec;
    if (!cv::solvePnP(objectPoints, corners, cameraMatrixToCv(intrinsics.cameraMatrix),
                      distortionToCv(intrinsics.distortionCoeffs), rvec, tvec, false,
                      cv::SOLVEPNP_ITERATIVE)) {
        result.status = ImageSampleStatus::PnpFailed;
        result.message = QStringLiteral("solvePnP failed for the detected board.");
        return result;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrixToCv(intrinsics.cameraMatrix),
                      distortionToCv(intrinsics.distortionCoeffs), projected);
    double squaredError = 0.0;
    for (size_t index = 0; index < corners.size(); ++index) {
        const cv::Point2f delta = projected[index] - corners[index];
        squaredError += static_cast<double>(delta.dot(delta));
    }

    result.success = true;
    result.status = ImageSampleStatus::PoseEstimated;
    result.detectedCornerCount = static_cast<int>(corners.size());
    result.reprojectionRmsePx = std::sqrt(squaredError / static_cast<double>(corners.size()));
    result.targetRotation = {rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)};
    result.targetTranslation = {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
    result.message = QStringLiteral("Chessboard pose estimated.");
    return result;
}

} // namespace handeye
