#pragma once

#include <QDateTime>
#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QVector>

#include <array>

namespace handeye {

using Matrix4 = std::array<std::array<double, 4>, 4>;
using Matrix3 = std::array<std::array<double, 3>, 3>;
using Vector2 = std::array<double, 2>;
using Vector3 = std::array<double, 3>;
using Vector4 = std::array<double, 4>;
using Vector5 = std::array<double, 5>;

enum class CalibrationMethod { Tsai, Park, Horaud, Andreff, Daniilidis, PointBased, Nonlinear };
enum class CalibrationMode { EyeInHand, EyeToHand };
enum class CalibrationInputMode { PosePairs, FixedPoint3D };
enum class RotationFormat { Rodrigues, EulerXYZ, RPY, QuaternionWXYZ };
enum class AngleUnit { Radians, Degrees };
enum class LengthUnit { Meters, Millimeters };
enum class PoseDirection { GripperToBase, TargetToCamera, CameraToGripper };
enum class PoseAdapterKind { Generic, UniversalRobots, Kuka, Fanuc };
enum class BoardPattern { Chessboard, Charuco, ArucoGrid };
enum class ChessboardDetector { Auto, Classic, SB };
enum class PnpMethod { Auto, Iterative, IPPE };
enum class ImageSampleStatus {
    NotProcessed,
    ImageMissing,
    DetectionFailed,
    PnpFailed,
    PoseEstimated,
    ManualPose
};
enum class CameraCalibrationSampleStatus { NotProcessed, DetectionFailed, Valid, Outlier };

struct PoseInputSpec {
    RotationFormat rotationFormat = RotationFormat::Rodrigues;
    AngleUnit angleUnit = AngleUnit::Radians;
    LengthUnit lengthUnit = LengthUnit::Meters;
    PoseAdapterKind adapter = PoseAdapterKind::Generic;
    PoseDirection direction = PoseDirection::GripperToBase;
    bool quaternionWFirst = true;
};

struct ManualPoseInput {
    int id = 0;
    Vector3 tcpTranslation{};
    Vector4 tcpRotation{};
    Vector3 cameraTranslation{};
    Vector4 cameraRotation{};
    QString label;
};

struct PointSample {
    int id = 0;
    Vector3 gripperRotation{};
    Vector3 gripperTranslation{};
    Vector3 cameraPoint{};
    QString label;
    Vector3 predictedBasePoint{};
    double residualM = 0.0;
    bool outlier = false;
};

struct SampleResidual {
    int sampleId = 0;
    double rotationErrorDeg = 0.0;
    double translationErrorM = 0.0;
    double normalizedScore = 0.0;
    int pairCount = 0;
    bool outlier = false;
};

struct BoardSpec {
    BoardPattern pattern = BoardPattern::Chessboard;
    ChessboardDetector chessboardDetector = ChessboardDetector::Auto;
    PnpMethod pnpMethod = PnpMethod::Auto;
    int innerCornersX = 9;
    int innerCornersY = 6;
    double squareSizeM = 0.025;
    int arucoDictionary = 0;
    int markerCountX = 5;
    int markerCountY = 7;
    double markerSizeM = 0.01875;
};

struct CameraIntrinsics {
    bool valid = false;
    Matrix3 cameraMatrix{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
    Vector5 distortionCoeffs{};
    int imageWidth = 0;
    int imageHeight = 0;
    QString source;
    QDateTime calibratedAt;
};

struct CameraCalibrationSample {
    QString imagePath;
    int imageWidth = 0;
    int imageHeight = 0;
    int detectedCornerCount = 0;
    double reprojectionRmsePx = 0.0;
    bool used = false;
    bool outlier = false;
    CameraCalibrationSampleStatus status = CameraCalibrationSampleStatus::NotProcessed;
    QString message;
    QVector<Vector2> corners;
    QVector<int> cornerIds;
    QVector<QVector<Vector2>> markerCorners;
    QString detectionMethod;
};

struct CameraCalibrationReport {
    bool available = false;
    bool success = false;
    bool passed = false;
    int initialImageCount = 0;
    int initialDetectedCount = 0;
    int finalUsedCount = 0;
    int outlierCount = 0;
    int imageWidth = 0;
    int imageHeight = 0;
    double outlierThresholdPx = 1.0;
    double rmsPx = 0.0;
    double meanRmsePx = 0.0;
    double maxRmsePx = 0.0;
    bool coverageWarning = false;
    QStringList errors;
    QStringList warnings;
    QString message;
    QDateTime calibratedAt;
    CameraIntrinsics intrinsics;
    QVector<CameraCalibrationSample> samples;
};

struct ReliabilityReport {
    bool available = false;
    bool valid = false;
    bool passed = false;
    int sampleCount = 0;
    int outlierCount = 0;
    double rotationRmseDeg = 0.0;
    double translationRmseM = 0.0;
    double rotationMeanDeg = 0.0;
    double translationMeanM = 0.0;
    double rotationMaxDeg = 0.0;
    double translationMaxM = 0.0;
    QStringList errors;
    QStringList warnings;
    QVector<SampleResidual> sampleResiduals;
};

struct FixedTargetPoseSample {
    int sampleId = 0;
    Matrix4 predictedPose{};
    Vector3 predictedRotation{};
    Vector3 predictedTranslation{};
    double rotationErrorToMeanDeg = 0.0;
    double translationErrorToMeanM = 0.0;
    double rotationErrorToReferenceDeg = 0.0;
    double translationErrorToReferenceM = 0.0;
    bool outlier = false;
};

struct FixedTargetPoseReport {
    bool available = false;
    bool success = false;
    int referenceSampleId = -1;
    Matrix4 robustMeanPose{};
    Vector3 robustMeanRotation{};
    Vector3 robustMeanTranslation{};
    double rotationRmseDeg = 0.0;
    double translationRmseM = 0.0;
    double rotationMeanDeg = 0.0;
    double translationMeanM = 0.0;
    double rotationMaxDeg = 0.0;
    double translationMaxM = 0.0;
    int outlierCount = 0;
    QStringList errors;
    QStringList warnings;
    QVector<FixedTargetPoseSample> samples;
};

struct FixedPointSample {
    int sampleId = 0;
    Vector3 predictedBasePoint{};
    double residualM = 0.0;
    bool outlier = false;
};

struct FixedPointReport {
    bool available = false;
    bool success = false;
    Vector3 robustMeanPoint{};
    double rmseM = 0.0;
    double meanErrorM = 0.0;
    double maxErrorM = 0.0;
    int outlierCount = 0;
    QStringList errors;
    QStringList warnings;
    QVector<FixedPointSample> samples;
};

struct PoseQualityReport {
    bool available = false;
    bool calculable = false;
    int sampleScore = 0;
    int rotationAmplitudeScore = 0;
    int rotationAxisScore = 0;
    int spatialDistributionScore = 0;
    int totalScore = 0;
    QString level;
    double maxRelativeRotationDeg = 0.0;
    int independentAxisCount = 0;
    bool nearMidFarCoverage = false;
    bool fullFovCoverage = false;
    bool imageCoverageAvailable = false;
    QStringList warnings;
};

struct NonlinearOptimizationReport {
    bool available = false;
    bool success = false;
    bool converged = false;
    int iterations = 0;
    int huberOutlierCount = 0;
    double beforeRotationRmseDeg = 0.0;
    double beforeTranslationRmseM = 0.0;
    double afterRotationRmseDeg = 0.0;
    double afterTranslationRmseM = 0.0;
    QString message;
};

// All PoseSample values are canonical: Rodrigues radians and meters.
struct PoseSample {
    int id = 0;
    Vector3 gripperRotation{};
    Vector3 gripperTranslation{};
    Vector3 targetRotation{};
    Vector3 targetTranslation{};
    QString label;
    double rotationResidualDeg = 0.0;
    double translationResidualM = 0.0;
    bool outlier = false;
    QString imagePath;
    ImageSampleStatus imageStatus = ImageSampleStatus::NotProcessed;
    int detectedCornerCount = 0;
    double pnpReprojectionRmsePx = 0.0;
    QString imageMessage;
    int imageWidth = 0;
    int imageHeight = 0;
    double imageCenterXNorm = 0.5;
    double imageCenterYNorm = 0.5;
    QString detectionMethod;
    PnpMethod selectedPnpMethod = PnpMethod::Auto;
    double iterativePnpRmsePx = 0.0;
    double ippePnpRmsePx = 0.0;
};

struct CalibrationResult {
    CalibrationMethod method = CalibrationMethod::Tsai;
    bool success = false;
    bool recommended = false;
    Matrix4 cameraToGripper{};
    double rotationErrorDeg = 0.0;
    double translationError = 0.0;
    qint64 elapsedMs = 0;
    QString message;
    ReliabilityReport trainingReport;
    ReliabilityReport validationReport;
    FixedTargetPoseReport fixedTargetReport;
    FixedPointReport fixedPointReport;
    PoseQualityReport qualityReport;
    NonlinearOptimizationReport optimizationReport;
};

struct CalibrationDataset {
    QVector<PoseSample> samples;
    QVector<PoseSample> validationSamples;
    bool targetPosesReady = false;
    CalibrationInputMode inputMode = CalibrationInputMode::PosePairs;
    QVector<PointSample> pointSamples;
    CalibrationMode mode = CalibrationMode::EyeInHand;
    PoseInputSpec inputSpec;
    QString robotName = QStringLiteral("未指定机器人");
    QString cameraName = QStringLiteral("未指定相机");
    QString notes;
    BoardSpec boardSpec;
    CameraIntrinsics cameraIntrinsics;
    CameraCalibrationReport cameraCalibrationReport;
    double passRotationRmseDeg = 0.5;
    double passTranslationRmseM = 0.001;
    bool hasGroundTruth = false;
    Matrix4 groundTruthCameraToGripper{};
    QVector<CalibrationResult> results;
    QDateTime createdAt = QDateTime::currentDateTime();
};

inline QString methodName(CalibrationMethod method)
{
    switch (method) {
    case CalibrationMethod::Tsai: return QStringLiteral("Tsai-Lenz");
    case CalibrationMethod::Park: return QStringLiteral("Park-Martin");
    case CalibrationMethod::Horaud: return QStringLiteral("Horaud");
    case CalibrationMethod::Andreff: return QStringLiteral("Andreff");
    case CalibrationMethod::Daniilidis: return QStringLiteral("Daniilidis");
    case CalibrationMethod::PointBased: return QStringLiteral("FixedPoint3D 点基");
    case CalibrationMethod::Nonlinear: return QStringLiteral("非线性精修");
    }
    return QStringLiteral("Unknown");
}

inline QString rotationFormatName(RotationFormat format)
{
    switch (format) {
    case RotationFormat::Rodrigues: return QStringLiteral("Rodrigues");
    case RotationFormat::EulerXYZ: return QStringLiteral("Euler XYZ");
    case RotationFormat::RPY: return QStringLiteral("RPY (Z-Y-X)");
    case RotationFormat::QuaternionWXYZ: return QStringLiteral("Quaternion (w,x,y,z)");
    }
    return QStringLiteral("Unknown");
}

inline QString angleUnitName(AngleUnit unit)
{
    return unit == AngleUnit::Radians ? QStringLiteral("rad") : QStringLiteral("deg");
}

inline QString lengthUnitName(LengthUnit unit)
{
    return unit == LengthUnit::Meters ? QStringLiteral("m") : QStringLiteral("mm");
}

inline QString directionName(PoseDirection direction)
{
    switch (direction) {
    case PoseDirection::GripperToBase: return QStringLiteral("gripper → base");
    case PoseDirection::TargetToCamera: return QStringLiteral("target → camera");
    case PoseDirection::CameraToGripper: return QStringLiteral("camera → gripper");
    }
    return QStringLiteral("Unknown");
}

inline QString inputModeName(CalibrationInputMode mode)
{
    return mode == CalibrationInputMode::FixedPoint3D
               ? QStringLiteral("FixedPoint3D 点基")
               : QStringLiteral("PosePairs 位姿对");
}

inline QString imageSampleStatusName(ImageSampleStatus status)
{
    switch (status) {
    case ImageSampleStatus::NotProcessed: return QStringLiteral("Not processed");
    case ImageSampleStatus::ImageMissing: return QStringLiteral("Image missing");
    case ImageSampleStatus::DetectionFailed: return QStringLiteral("Board detection failed");
    case ImageSampleStatus::PnpFailed: return QStringLiteral("PnP failed");
    case ImageSampleStatus::PoseEstimated: return QStringLiteral("Pose estimated");
    case ImageSampleStatus::ManualPose: return QStringLiteral("手动输入位姿");
    }
    return QStringLiteral("Unknown");
}

inline ImageSampleStatus imageSampleStatusFromName(const QString &name)
{
    if (name == QStringLiteral("Image missing")) return ImageSampleStatus::ImageMissing;
    if (name == QStringLiteral("Board detection failed")) return ImageSampleStatus::DetectionFailed;
    if (name == QStringLiteral("PnP failed")) return ImageSampleStatus::PnpFailed;
    if (name == QStringLiteral("Pose estimated")) return ImageSampleStatus::PoseEstimated;
    if (name == QStringLiteral("手动输入位姿")) return ImageSampleStatus::ManualPose;
    return ImageSampleStatus::NotProcessed;
}

inline QString cameraCalibrationSampleStatusName(CameraCalibrationSampleStatus status)
{
    switch (status) {
    case CameraCalibrationSampleStatus::NotProcessed: return QStringLiteral("未处理");
    case CameraCalibrationSampleStatus::DetectionFailed: return QStringLiteral("角点检测失败");
    case CameraCalibrationSampleStatus::Valid: return QStringLiteral("有效");
    case CameraCalibrationSampleStatus::Outlier: return QStringLiteral("异常剔除");
    }
    return QStringLiteral("未知");
}

inline CameraCalibrationSampleStatus cameraCalibrationSampleStatusFromName(const QString &name)
{
    if (name == QStringLiteral("角点检测失败")) return CameraCalibrationSampleStatus::DetectionFailed;
    if (name == QStringLiteral("有效")) return CameraCalibrationSampleStatus::Valid;
    if (name == QStringLiteral("异常剔除")) return CameraCalibrationSampleStatus::Outlier;
    return CameraCalibrationSampleStatus::NotProcessed;
}

inline QString boardPatternName(BoardPattern pattern)
{
    switch (pattern) {
    case BoardPattern::Chessboard: return QStringLiteral("Chessboard");
    case BoardPattern::Charuco: return QStringLiteral("ChArUco");
    case BoardPattern::ArucoGrid: return QStringLiteral("ArUco Grid");
    }
    return QStringLiteral("Unknown");
}

inline QString chessboardDetectorName(ChessboardDetector detector)
{
    switch (detector) {
    case ChessboardDetector::Auto: return QStringLiteral("Auto (SB 优先)");
    case ChessboardDetector::Classic: return QStringLiteral("Classic");
    case ChessboardDetector::SB: return QStringLiteral("SB");
    }
    return QStringLiteral("Unknown");
}

inline QString pnpMethodName(PnpMethod method)
{
    switch (method) {
    case PnpMethod::Auto: return QStringLiteral("Auto (ITERATIVE/IPPE)");
    case PnpMethod::Iterative: return QStringLiteral("ITERATIVE");
    case PnpMethod::IPPE: return QStringLiteral("IPPE");
    }
    return QStringLiteral("Unknown");
}

inline QVector<CalibrationMethod> allMethods()
{
    return {CalibrationMethod::Tsai, CalibrationMethod::Park, CalibrationMethod::Horaud,
            CalibrationMethod::Andreff, CalibrationMethod::Daniilidis};
}

} // namespace handeye

Q_DECLARE_METATYPE(handeye::CalibrationResult)
