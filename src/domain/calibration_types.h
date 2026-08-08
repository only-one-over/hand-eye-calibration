#pragma once

#include <QDateTime>
#include <QMetaType>
#include <QString>
#include <QVector>

#include <array>

namespace handeye {

using Matrix4 = std::array<std::array<double, 4>, 4>;
using Vector3 = std::array<double, 3>;

enum class CalibrationMethod { Tsai, Park, Horaud, Andreff, Daniilidis };
enum class CalibrationMode { EyeInHand, EyeToHand };

struct PoseSample {
    int id = 0;
    Vector3 gripperRotation{};
    Vector3 gripperTranslation{};
    Vector3 targetRotation{};
    Vector3 targetTranslation{};
    QString label;
};

struct CalibrationResult {
    CalibrationMethod method = CalibrationMethod::Tsai;
    bool success = false;
    Matrix4 cameraToGripper{};
    double rotationErrorDeg = 0.0;
    double translationError = 0.0;
    qint64 elapsedMs = 0;
    QString message;
};

struct CalibrationDataset {
    QVector<PoseSample> samples;
    CalibrationMode mode = CalibrationMode::EyeInHand;
    QString translationUnit = QStringLiteral("m");
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

inline QVector<CalibrationMethod> allMethods()
{
    return {CalibrationMethod::Tsai, CalibrationMethod::Park, CalibrationMethod::Horaud,
            CalibrationMethod::Andreff, CalibrationMethod::Daniilidis};
}

} // namespace handeye

Q_DECLARE_METATYPE(handeye::CalibrationResult)
