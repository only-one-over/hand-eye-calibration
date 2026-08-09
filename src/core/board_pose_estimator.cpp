#include "core/board_pose_estimator.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

#include <cmath>
#include <limits>

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

cv::aruco::Dictionary dictionaryFor(const BoardSpec &board)
{
    return cv::aruco::getPredefinedDictionary(board.arucoDictionary);
}

BoardCornerDetection detectArucoBoard(const cv::Mat &image, const BoardSpec &board)
{
    BoardCornerDetection result;
    const cv::aruco::Dictionary dictionary = dictionaryFor(board);
    const bool validGeometry = board.pattern == BoardPattern::Charuco
                                   ? board.innerCornersX >= 1 && board.innerCornersY >= 1
                                         && board.squareSizeM > 0.0 && board.markerSizeM > 0.0
                                         && board.markerSizeM < board.squareSizeM
                                   : board.markerCountX >= 1 && board.markerCountY >= 1
                                         && board.markerSizeM > 0.0
                                         && board.markerSeparationM >= 0.0;
    if (!validGeometry) {
        result.status = ImageSampleStatus::DetectionFailed;
        result.message = QStringLiteral("标定板几何参数无效，请检查网格尺寸、marker 尺寸和间距。");
        return result;
    }
    cv::aruco::DetectorParameters detectorParameters;
    cv::aruco::ArucoDetector detector(dictionary, detectorParameters);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    detector.detectMarkers(image, markerCorners, ids);
    if (ids.empty()) {
        result.status = ImageSampleStatus::DetectionFailed;
        result.message = QStringLiteral("未检测到 ArUco marker。");
        return result;
    }

    if (board.pattern == BoardPattern::Charuco) {
        const cv::aruco::CharucoBoard charucoBoard(
            cv::Size(board.innerCornersX + 1, board.innerCornersY + 1),
            static_cast<float>(board.squareSizeM), static_cast<float>(board.markerSizeM), dictionary);
        cv::aruco::CharucoDetector charucoDetector(charucoBoard);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        charucoDetector.detectBoard(image, charucoCorners, charucoIds, markerCorners, ids);
        if (charucoCorners.size() < 4) {
            result.status = ImageSampleStatus::DetectionFailed;
            result.message = QStringLiteral("ChArUco 有效角点少于 4 个。");
            return result;
        }
        result.corners.reserve(static_cast<int>(charucoCorners.size()));
        for (const cv::Point2f &corner : charucoCorners) result.corners.append({corner.x, corner.y});
        for (int id : charucoIds) result.cornerIds.append(id);
        result.detectionMethod = QStringLiteral("ChArUco marker + 亚像素角点");
    } else {
        if (board.markerCountX < 1 || board.markerCountY < 1 || board.markerSizeM <= 0.0
            || board.markerSeparationM < 0.0) {
            result.status = ImageSampleStatus::DetectionFailed;
            result.message = QStringLiteral("ArUco Grid 参数无效。");
            return result;
        }
        result.markerCorners.reserve(static_cast<int>(markerCorners.size()));
        for (int markerIndex = 0; markerIndex < static_cast<int>(markerCorners.size()); ++markerIndex) {
            QVector<Vector2> marker;
            for (const cv::Point2f &corner : markerCorners[markerIndex]) {
                marker.append({corner.x, corner.y});
                result.corners.append({corner.x, corner.y});
            }
            result.markerCorners.append(marker);
            result.cornerIds.append(ids[markerIndex]);
        }
        result.detectionMethod = QStringLiteral("ArUco Grid marker");
    }
    result.success = true;
    result.status = ImageSampleStatus::PoseEstimated;
    result.message = QStringLiteral("%1 检测成功。").arg(result.detectionMethod);
    return result;
}

std::vector<cv::Point3f> chessObjectPoints(const BoardSpec &board)
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<size_t>(board.innerCornersX * board.innerCornersY));
    for (int row = 0; row < board.innerCornersY; ++row)
        for (int col = 0; col < board.innerCornersX; ++col)
            points.emplace_back(static_cast<float>(col * board.squareSizeM),
                                static_cast<float>(row * board.squareSizeM), 0.0F);
    return points;
}

void makeObjectAndImagePoints(const BoardCornerDetection &detection,
                              const BoardSpec &board,
                              std::vector<cv::Point3f> *objectPoints,
                              std::vector<cv::Point2f> *imagePoints)
{
    if (!objectPoints || !imagePoints) return;
    *imagePoints = toCvCorners(detection.corners);
    if (board.pattern == BoardPattern::Chessboard) {
        *objectPoints = chessObjectPoints(board);
        return;
    }
    const cv::aruco::Dictionary dictionary = dictionaryFor(board);
    if (board.pattern == BoardPattern::Charuco) {
        const cv::aruco::CharucoBoard charucoBoard(
            cv::Size(board.innerCornersX + 1, board.innerCornersY + 1),
            static_cast<float>(board.squareSizeM), static_cast<float>(board.markerSizeM), dictionary);
        const std::vector<cv::Point3f> boardCorners = charucoBoard.getChessboardCorners();
        for (int id : detection.cornerIds)
            if (id >= 0 && id < static_cast<int>(boardCorners.size())) objectPoints->push_back(boardCorners[id]);
        return;
    }
    const cv::aruco::GridBoard gridBoard(
        cv::Size(board.markerCountX, board.markerCountY), static_cast<float>(board.markerSizeM),
        static_cast<float>(board.markerSeparationM), dictionary);
    std::vector<std::vector<cv::Point2f>> markerCorners;
    markerCorners.reserve(static_cast<size_t>(detection.markerCorners.size()));
    for (const QVector<Vector2> &marker : detection.markerCorners) markerCorners.push_back(toCvCorners(marker));
    std::vector<int> markerIds;
    for (int id : detection.cornerIds) markerIds.push_back(id);
    gridBoard.matchImagePoints(markerCorners, markerIds, *objectPoints, *imagePoints);
}

struct PnpCandidate {
    bool success = false;
    PnpMethod method = PnpMethod::Auto;
    cv::Mat rvec;
    cv::Mat tvec;
    double rmse = std::numeric_limits<double>::infinity();
};

PnpCandidate tryPnp(const std::vector<cv::Point3f> &objectPoints,
                   const std::vector<cv::Point2f> &imagePoints,
                   const cv::Mat &cameraMatrix,
                   const cv::Mat &distortion,
                   PnpMethod method)
{
    PnpCandidate result;
    result.method = method;
    if (objectPoints.size() < 4 || imagePoints.size() != objectPoints.size()) return result;
    cv::Mat rvec, tvec;
    try {
        const int flag = method == PnpMethod::IPPE ? cv::SOLVEPNP_IPPE : cv::SOLVEPNP_ITERATIVE;
        if (!cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distortion, rvec, tvec, false, flag)) return result;
        cv::Mat rotation;
        cv::Rodrigues(rvec, rotation);
        for (const cv::Point3f &point : objectPoints) {
            const double z = rotation.at<double>(2, 0) * point.x + rotation.at<double>(2, 1) * point.y
                             + rotation.at<double>(2, 2) * point.z + tvec.at<double>(2);
            if (z <= 0.0) return result;
        }
        std::vector<cv::Point2f> projected;
        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortion, projected);
        double squared = 0.0;
        for (size_t index = 0; index < projected.size(); ++index) {
            const cv::Point2f delta = projected[index] - imagePoints[index];
            squared += static_cast<double>(delta.dot(delta));
        }
        result.success = true;
        result.rvec = rvec;
        result.tvec = tvec;
        result.rmse = std::sqrt(squared / static_cast<double>(projected.size()));
    } catch (const cv::Exception &) {
    }
    return result;
}

} // namespace

BoardCornerDetection BoardPoseEstimator::detectChessboard(const QString &imagePath, const BoardSpec &board)
{
    BoardCornerDetection result;
    if (imagePath.trimmed().isEmpty()) {
        result.status = ImageSampleStatus::ImageMissing;
        result.message = QStringLiteral("图片路径为空。");
        return result;
    }
    const cv::Mat image = cv::imread(imagePath.toLocal8Bit().constData(), cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        result.status = ImageSampleStatus::ImageMissing;
        result.message = QStringLiteral("无法加载图片：%1").arg(imagePath);
        return result;
    }
    result.imageWidth = image.cols;
    result.imageHeight = image.rows;

    if (board.pattern != BoardPattern::Chessboard)
        return [&] {
            BoardCornerDetection detected = detectArucoBoard(image, board);
            detected.imageWidth = image.cols;
            detected.imageHeight = image.rows;
            return detected;
        }();

    if (board.innerCornersX < 2 || board.innerCornersY < 2 || board.squareSizeM <= 0.0) {
        result.status = ImageSampleStatus::DetectionFailed;
        result.message = QStringLiteral("棋盘格行列数和方格尺寸必须为正。");
        return result;
    }
    const cv::Size patternSize(board.innerCornersX, board.innerCornersY);
    std::vector<cv::Point2f> corners;
    bool detected = false;
    if (board.chessboardDetector != ChessboardDetector::Classic)
        detected = cv::findChessboardCornersSB(image, patternSize, corners, cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!detected && board.chessboardDetector != ChessboardDetector::SB) {
        corners.clear();
        detected = cv::findChessboardCorners(image, patternSize, corners,
                                             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (detected) {
            cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            result.detectionMethod = QStringLiteral("Chessboard Classic + cornerSubPix");
        }
    } else if (detected) {
        result.detectionMethod = QStringLiteral("Chessboard SB");
    }
    if (!detected) {
        result.status = ImageSampleStatus::DetectionFailed;
        result.message = QStringLiteral("未检测到棋盘格角点。");
        return result;
    }
    result.corners.reserve(static_cast<int>(corners.size()));
    for (const cv::Point2f &corner : corners) result.corners.append({corner.x, corner.y});
    result.success = true;
    result.status = ImageSampleStatus::PoseEstimated;
    result.message = QStringLiteral("%1 检测成功。").arg(result.detectionMethod);
    return result;
}

BoardPoseEstimate BoardPoseEstimator::estimateChessboard(const QString &imagePath,
                                                         const BoardSpec &board,
                                                         const CameraIntrinsics &intrinsics)
{
    BoardPoseEstimate result;
    if (!intrinsics.valid) {
        result.status = ImageSampleStatus::PnpFailed;
        result.message = QStringLiteral("solvePnP 需要有效的相机内参。");
        return result;
    }
    const BoardCornerDetection detection = detectChessboard(imagePath, board);
    if (!detection.success) {
        result.status = detection.status;
        result.message = detection.message;
        return result;
    }
    result.imageWidth = detection.imageWidth;
    result.imageHeight = detection.imageHeight;
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    makeObjectAndImagePoints(detection, board, &objectPoints, &imagePoints);
    const cv::Mat cameraMatrix = cameraMatrixToCv(intrinsics.cameraMatrix);
    const cv::Mat distortion = distortionToCv(intrinsics.distortionCoeffs);
    QVector<PnpCandidate> candidates;
    if (board.pnpMethod != PnpMethod::IPPE)
        candidates.append(tryPnp(objectPoints, imagePoints, cameraMatrix, distortion, PnpMethod::Iterative));
    if (board.pnpMethod != PnpMethod::Iterative)
        candidates.append(tryPnp(objectPoints, imagePoints, cameraMatrix, distortion, PnpMethod::IPPE));
    PnpCandidate best;
    for (const PnpCandidate &candidate : candidates) {
        if (candidate.success && (!best.success || candidate.rmse < best.rmse)) best = candidate;
        if (candidate.method == PnpMethod::Iterative && candidate.success) result.iterativePnpRmsePx = candidate.rmse;
        if (candidate.method == PnpMethod::IPPE && candidate.success) result.ippePnpRmsePx = candidate.rmse;
    }
    if (!best.success) {
        result.status = ImageSampleStatus::PnpFailed;
        result.message = QStringLiteral("ITERATIVE/IPPE 均未能求解有效位姿。");
        return result;
    }
    result.success = true;
    result.status = ImageSampleStatus::PoseEstimated;
    result.detectedCornerCount = static_cast<int>(imagePoints.size());
    result.reprojectionRmsePx = best.rmse;
    result.selectedPnpMethod = best.method;
    result.detectionMethod = detection.detectionMethod;
    result.targetRotation = {best.rvec.at<double>(0), best.rvec.at<double>(1), best.rvec.at<double>(2)};
    result.targetTranslation = {best.tvec.at<double>(0), best.tvec.at<double>(1), best.tvec.at<double>(2)};
    result.message = QStringLiteral("%1；PnP 选择 %2，RMSE %3 px。")
                         .arg(detection.detectionMethod, pnpMethodName(best.method))
                         .arg(best.rmse, 0, 'f', 3);
    return result;
}

} // namespace handeye
