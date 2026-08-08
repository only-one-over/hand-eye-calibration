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
Vector4 rotationToFormat(const Vector3 &rodriguesRadians, RotationFormat format,
                         AngleUnit angleUnit);
PoseInputSpec defaultSpec(PoseAdapterKind adapter);

} // namespace handeye::pose
