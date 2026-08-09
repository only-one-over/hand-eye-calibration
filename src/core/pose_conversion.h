#pragma once

#include "domain/calibration_types.h"

#include <QString>
#include <QVector>

namespace handeye::pose {

struct NormalizationResult {
    bool success = false;
    Vector3 rotation{};
    Vector3 translation{};
    QString error;
};

NormalizationResult normalize(const Vector4 &rotation, const Vector3 &translation,
                              const PoseInputSpec &spec);
NormalizationResult normalizeKukaAbc(const Vector4 &rotation, const Vector3 &translation,
                                     AngleUnit angleUnit = AngleUnit::Degrees,
                                     LengthUnit lengthUnit = LengthUnit::Millimeters);
NormalizationResult normalizeFanucWpr(const Vector4 &rotation, const Vector3 &translation,
                                      AngleUnit angleUnit = AngleUnit::Degrees,
                                      LengthUnit lengthUnit = LengthUnit::Millimeters);
Vector4 rotationToFormat(const Vector3 &rodriguesRadians, RotationFormat format,
                         AngleUnit angleUnit);
Vector4 rotationToConvention(const Vector3 &rodriguesRadians, PoseConvention convention,
                             AngleUnit angleUnit);
PoseInputSpec defaultSpec(PoseAdapterKind adapter);

} // namespace handeye::pose
