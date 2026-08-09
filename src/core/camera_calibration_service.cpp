#include "core/camera_calibration_service.h"

#include "core/board_pose_estimator.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace handeye {

namespace {

struct FitResult {
    bool success = false;
    double rmsPx = 0.0;
    cv::Mat cameraMatrix;
    cv::Mat distortion;
    QVector<double> viewRmse;
    QString error;
};

std::vector<cv::Point3f> makeChessObjectPoints(const BoardSpec &board)
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<size_t>(board.innerCornersX * board.innerCornersY));
    for (int row = 0; row < board.innerCornersY; ++row) {
        for (int col = 0; col < board.innerCornersX; ++col) {
            points.emplace_back(static_cast<float>(col * board.squareSizeM),
                                static_cast<float>(row * board.squareSizeM), 0.0F);
        }
    }
    return points;
}

std::vector<cv::Point2f> toCvPoints(const QVector<Vector2> &points);

bool makeViewPoints(const CameraCalibrationSample &sample,
                    const BoardSpec &board,
                    std::vector<cv::Point3f> *objectPoints,
                    std::vector<cv::Point2f> *imagePoints)
{
    if (!objectPoints || !imagePoints) return false;
    objectPoints->clear();
    imagePoints->clear();
    if (board.pattern == BoardPattern::Chessboard) {
        *objectPoints = makeChessObjectPoints(board);
        *imagePoints = toCvPoints(sample.corners);
        return objectPoints->size() == imagePoints->size();
    }
    const cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(board.arucoDictionary);
    if (board.pattern == BoardPattern::Charuco) {
        const cv::aruco::CharucoBoard charucoBoard(
            cv::Size(board.innerCornersX + 1, board.innerCornersY + 1),
            static_cast<float>(board.squareSizeM), static_cast<float>(board.markerSizeM), dictionary);
        const auto boardCorners = charucoBoard.getChessboardCorners();
        *imagePoints = toCvPoints(sample.corners);
        for (int id : sample.cornerIds)
            if (id >= 0 && id < static_cast<int>(boardCorners.size())) objectPoints->push_back(boardCorners[id]);
        return objectPoints->size() == imagePoints->size() && objectPoints->size() >= 4;
    }
    const cv::aruco::GridBoard gridBoard(
        cv::Size(board.markerCountX, board.markerCountY), static_cast<float>(board.markerSizeM),
        static_cast<float>(board.markerSizeM * 0.25), dictionary);
    std::vector<std::vector<cv::Point2f>> markerCorners;
    for (const QVector<Vector2> &marker : sample.markerCorners) markerCorners.push_back(toCvPoints(marker));
    std::vector<int> ids;
    for (int id : sample.cornerIds) ids.push_back(id);
    gridBoard.matchImagePoints(markerCorners, ids, *objectPoints, *imagePoints);
    return objectPoints->size() == imagePoints->size() && objectPoints->size() >= 4;
}

std::vector<cv::Point2f> toCvPoints(const QVector<Vector2> &points)
{
    std::vector<cv::Point2f> result;
    result.reserve(static_cast<size_t>(points.size()));
    for (const Vector2 &point : points)
        result.emplace_back(static_cast<float>(point[0]), static_cast<float>(point[1]));
    return result;
}

Matrix3 matrixFromCv(const cv::Mat &matrix)
{
    Matrix3 result{};
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            result[row][col] = matrix.at<double>(row, col);
    return result;
}

Vector5 distortionFromCv(const cv::Mat &distortion)
{
    Vector5 result{};
    const cv::Mat flat = distortion.reshape(1, 1);
    for (int index = 0; index < 5 && index < flat.cols; ++index)
        result[index] = flat.at<double>(0, index);
    return result;
}

double viewRmse(const std::vector<cv::Point3f> &objectPoints,
                const std::vector<cv::Point2f> &imagePoints,
                const cv::Mat &rvec,
                const cv::Mat &tvec,
                const cv::Mat &cameraMatrix,
                const cv::Mat &distortion)
{
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortion, projected);
    if (projected.empty() || projected.size() != imagePoints.size())
        return std::numeric_limits<double>::infinity();

    double squaredError = 0.0;
    for (size_t index = 0; index < projected.size(); ++index) {
        const cv::Point2f delta = projected[index] - imagePoints[index];
        squaredError += static_cast<double>(delta.dot(delta));
    }
    return std::sqrt(squaredError / static_cast<double>(projected.size()));
}

FitResult fit(const QVector<CameraCalibrationSample> &samples,
              const QVector<int> &indices,
              const BoardSpec &board,
              int imageWidth,
              int imageHeight)
{
    FitResult result;
    if (indices.isEmpty()) {
        result.error = QStringLiteral("没有可用于标定的图片。");
        return result;
    }

    std::vector<std::vector<cv::Point3f>> objectViews;
    std::vector<std::vector<cv::Point2f>> imageViews;
    objectViews.reserve(static_cast<size_t>(indices.size()));
    imageViews.reserve(static_cast<size_t>(indices.size()));
    for (int index : indices) {
        const CameraCalibrationSample &sample = samples.at(index);
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        if (!makeViewPoints(sample, board, &objectPoints, &imagePoints)) {
            result.error = QStringLiteral("第 %1 张图片的标定板点数或 ID 无效。").arg(index + 1);
            return result;
        }
        objectViews.push_back(objectPoints);
        imageViews.push_back(imagePoints);
    }

    try {
        result.cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        result.distortion = cv::Mat::zeros(1, 5, CV_64F);
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;
        result.rmsPx = cv::calibrateCamera(objectViews, imageViews, cv::Size(imageWidth, imageHeight),
                                            result.cameraMatrix, result.distortion, rvecs, tvecs, 0);
        result.viewRmse.reserve(indices.size());
        for (int viewIndex = 0; viewIndex < indices.size(); ++viewIndex)
            result.viewRmse.append(viewRmse(objectViews.at(static_cast<size_t>(viewIndex)),
                                            imageViews.at(static_cast<size_t>(viewIndex)),
                                            rvecs.at(static_cast<size_t>(viewIndex)),
                                            tvecs.at(static_cast<size_t>(viewIndex)),
                                            result.cameraMatrix, result.distortion));
        result.success = true;
    } catch (const cv::Exception &exception) {
        result.error = QStringLiteral("OpenCV 相机标定失败：%1").arg(QString::fromUtf8(exception.what()));
    }
    return result;
}

void setSummary(CameraCalibrationReport &report, const QVector<int> &indices,
                const QVector<double> &rmseValues)
{
    report.finalUsedCount = indices.size();
    if (rmseValues.isEmpty()) return;
    const double sum = std::accumulate(rmseValues.cbegin(), rmseValues.cend(), 0.0);
    report.meanRmsePx = sum / static_cast<double>(rmseValues.size());
    report.maxRmsePx = *std::max_element(rmseValues.cbegin(), rmseValues.cend());
}

void evaluateCoverage(CameraCalibrationReport &report,
                       const QVector<CameraCalibrationSample> &samples,
                       const QVector<int> &indices)
{
    if (indices.size() < 10) {
        report.coverageWarning = true;
        report.warnings.append(QStringLiteral("有效图片少于 10 张，建议增加不同姿态的标定板图片。"));
    }
    if (indices.size() < 2 || report.imageWidth <= 0 || report.imageHeight <= 0) return;

    double minX = 1.0;
    double maxX = 0.0;
    double minY = 1.0;
    double maxY = 0.0;
    for (int index : indices) {
        const auto &corners = samples.at(index).corners;
        if (corners.isEmpty()) continue;
        double centerX = 0.0;
        double centerY = 0.0;
        for (const Vector2 &corner : corners) {
            centerX += corner[0];
            centerY += corner[1];
        }
        centerX /= static_cast<double>(corners.size() * report.imageWidth);
        centerY /= static_cast<double>(corners.size() * report.imageHeight);
        minX = std::min(minX, centerX);
        maxX = std::max(maxX, centerX);
        minY = std::min(minY, centerY);
        maxY = std::max(maxY, centerY);
    }
    if ((maxX - minX) < 0.20 && (maxY - minY) < 0.20) {
        report.coverageWarning = true;
        report.warnings.append(QStringLiteral("棋盘格中心位置变化较小，建议增加不同位置和倾斜角度的图片。"));
    }
}

CameraCalibrationReport calibrateDetected(CameraCalibrationReport report,
                                           const BoardSpec &board,
                                           const CameraCalibrationOptions &options)
{
    report.outlierThresholdPx = options.outlierThresholdPx;
    report.initialImageCount = report.samples.size();
    report.initialDetectedCount = 0;
    report.outlierCount = 0;
    report.success = false;
    report.passed = false;
    report.errors.clear();
    report.message.clear();

    QVector<int> validIndices;
    for (int index = 0; index < report.samples.size(); ++index) {
        auto &sample = report.samples[index];
        sample.used = false;
        sample.outlier = false;
        if (sample.status == CameraCalibrationSampleStatus::Valid && !sample.corners.isEmpty()) {
            validIndices.append(index);
            ++report.initialDetectedCount;
        }
    }
    if (report.initialDetectedCount < options.minValidViews) {
        report.errors.append(QStringLiteral("至少需要 %1 张成功检测棋盘格的图片，当前只有 %2 张。")
                                 .arg(options.minValidViews).arg(report.initialDetectedCount));
        report.message = QStringLiteral("图片数量不足，无法进行相机标定。");
        return report;
    }

    int width = 0;
    int height = 0;
    for (int index : validIndices) {
        const auto &sample = report.samples.at(index);
        if (width == 0) {
            width = sample.imageWidth;
            height = sample.imageHeight;
        } else if (sample.imageWidth != width || sample.imageHeight != height) {
            report.errors.append(QStringLiteral("所有标定图片必须具有相同分辨率。"));
            report.message = QStringLiteral("图片分辨率不一致，无法进行相机标定。");
            return report;
        }
    }
    report.imageWidth = width;
    report.imageHeight = height;

    FitResult initial = fit(report.samples, validIndices, board, width, height);
    if (!initial.success) {
        report.errors.append(initial.error);
        report.message = initial.error;
        return report;
    }

    QVector<int> finalIndices;
    QVector<double> finalRmse;
    for (int position = 0; position < validIndices.size(); ++position) {
        const int sampleIndex = validIndices.at(position);
        auto &sample = report.samples[sampleIndex];
        sample.reprojectionRmsePx = initial.viewRmse.at(position);
        if (sample.reprojectionRmsePx > options.outlierThresholdPx) {
            sample.outlier = true;
            sample.used = false;
            sample.status = CameraCalibrationSampleStatus::Outlier;
            sample.message = QStringLiteral("单图重投影 RMSE 超过 %1 px。")
                                 .arg(options.outlierThresholdPx, 0, 'f', 3);
            ++report.outlierCount;
        } else {
            finalIndices.append(sampleIndex);
            sample.used = true;
            sample.message = QStringLiteral("参与最终标定。");
        }
    }

    FitResult finalFit = initial;
    if (report.outlierCount > 0) {
        if (finalIndices.size() < options.minValidViews) {
            report.errors.append(QStringLiteral("剔除异常图片后有效图片少于 %1 张。")
                                     .arg(options.minValidViews));
            report.message = QStringLiteral("异常图片过多，无法生成可靠内参。");
            return report;
        }
        finalFit = fit(report.samples, finalIndices, board, width, height);
        if (!finalFit.success) {
            report.errors.append(finalFit.error);
            report.message = finalFit.error;
            return report;
        }
        for (int position = 0; position < finalIndices.size(); ++position)
            report.samples[finalIndices.at(position)].reprojectionRmsePx = finalFit.viewRmse.at(position);
    } else {
        finalIndices = validIndices;
    }

    finalRmse.reserve(finalIndices.size());
    for (int index : finalIndices) finalRmse.append(report.samples.at(index).reprojectionRmsePx);
    report.rmsPx = finalFit.rmsPx;
    setSummary(report, finalIndices, finalRmse);
    evaluateCoverage(report, report.samples, finalIndices);
    report.intrinsics.valid = true;
    report.intrinsics.cameraMatrix = matrixFromCv(finalFit.cameraMatrix);
    report.intrinsics.distortionCoeffs = distortionFromCv(finalFit.distortion);
    report.intrinsics.imageWidth = width;
    report.intrinsics.imageHeight = height;
    report.intrinsics.source = QStringLiteral("相机自主标定");
    report.intrinsics.calibratedAt = QDateTime::currentDateTime();
    report.calibratedAt = report.intrinsics.calibratedAt;
    report.success = true;
    report.passed = report.rmsPx <= options.outlierThresholdPx;
    report.message = QStringLiteral("相机标定完成：使用 %1/%2 张图片，RMS %3 px。")
                         .arg(report.finalUsedCount).arg(report.initialImageCount)
                         .arg(report.rmsPx, 0, 'f', 4);
    if (!report.passed)
        report.warnings.append(QStringLiteral("总体 RMS 超过当前异常阈值，请检查图片质量。"));
    return report;
}

} // namespace

CameraCalibrationReport CameraCalibrationService::detectImages(const QStringList &imagePaths,
                                                               const BoardSpec &board)
{
    CameraCalibrationReport report;
    report.available = true;
    report.initialImageCount = imagePaths.size();
    for (const QString &path : imagePaths) {
        CameraCalibrationSample sample;
        sample.imagePath = path;
        const BoardCornerDetection detection = BoardPoseEstimator::detectChessboard(path, board);
        sample.imageWidth = detection.imageWidth;
        sample.imageHeight = detection.imageHeight;
        sample.corners = detection.corners;
        sample.cornerIds = detection.cornerIds;
        sample.markerCorners = detection.markerCorners;
        sample.detectionMethod = detection.detectionMethod;
        sample.detectedCornerCount = detection.corners.size();
        sample.message = detection.message;
        if (detection.success) {
            sample.status = CameraCalibrationSampleStatus::Valid;
            sample.used = true;
            ++report.initialDetectedCount;
        } else {
            sample.status = CameraCalibrationSampleStatus::DetectionFailed;
        }
        report.samples.append(sample);
        if (report.imageWidth == 0 && sample.imageWidth > 0) {
            report.imageWidth = sample.imageWidth;
            report.imageHeight = sample.imageHeight;
        }
    }
    report.message = QStringLiteral("角点检测完成：%1/%2 张成功。")
                         .arg(report.initialDetectedCount).arg(report.initialImageCount);
    return report;
}

CameraCalibrationReport CameraCalibrationService::calibrateImages(const QStringList &imagePaths,
                                                                  const BoardSpec &board,
                                                                  const CameraCalibrationOptions &options)
{
    return calibrateDetected(detectImages(imagePaths, board), board, options);
}

CameraCalibrationReport CameraCalibrationService::calibrateObservations(
    const QVector<CameraCalibrationSample> &samples,
    const BoardSpec &board,
    const CameraCalibrationOptions &options)
{
    CameraCalibrationReport report;
    report.available = true;
    report.samples = samples;
    report.initialImageCount = samples.size();
    return calibrateDetected(report, board, options);
}

} // namespace handeye
