#pragma once

#include <QDateTime>
#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QVector>

#include <array>

namespace handeye {

using Matrix4 = std::array<std::array<double, 4>, 4>;
using Vector3 = std::array<double, 3>;
using Vector4 = std::array<double, 4>;

enum class CalibrationMethod { Tsai, Park, Horaud, Andreff, Daniilidis };
enum class CalibrationMode { EyeInHand, EyeToHand };
enum class RotationFormat { Rodrigues, EulerXYZ, RPY, QuaternionWXYZ };
enum class AngleUnit { Radians, Degrees };
enum class LengthUnit { Meters, Millimeters };
enum class PoseDirection { GripperToBase, TargetToCamera, CameraToGripper };
enum class PoseAdapterKind { Generic, UniversalRobots, Kuka, Fanuc };

struct PoseInputSpec {
    RotationFormat rotationFormat = RotationFormat::Rodrigues;
    AngleUnit angleUnit = AngleUnit::Radians;
    LengthUnit lengthUnit = LengthUnit::Meters;
    PoseAdapterKind adapter = PoseAdapterKind::Generic;
    PoseDirection direction = PoseDirection::GripperToBase;
    bool quaternionWFirst = true;
};

struct SampleResidual {
    int sampleId = 0;
    double rotationErrorDeg = 0.0;
    double translationErrorM = 0.0;
    double normalizedScore = 0.0;
    int pairCount = 0;
    bool outlier = false;
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
};

struct CalibrationDataset {
    QVector<PoseSample> samples;
    QVector<PoseSample> validationSamples;
    CalibrationMode mode = CalibrationMode::EyeInHand;
    PoseInputSpec inputSpec;
    QString robotName = QStringLiteral("未指定机器人");
    QString cameraName = QStringLiteral("未指定相机");
    QString notes;
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

inline QVector<CalibrationMethod> allMethods()
{
    return {CalibrationMethod::Tsai, CalibrationMethod::Park, CalibrationMethod::Horaud,
            CalibrationMethod::Andreff, CalibrationMethod::Daniilidis};
}

} // namespace handeye

Q_DECLARE_METATYPE(handeye::CalibrationResult)
