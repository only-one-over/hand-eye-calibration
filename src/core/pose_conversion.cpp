#include "core/pose_conversion.h"

#include "core/matrix_utils.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>

namespace handeye::pose {

namespace {

cv::Matx33d rx(double angle)
{
    const double c = std::cos(angle), s = std::sin(angle);
    return cv::Matx33d(1, 0, 0, 0, c, -s, 0, s, c);
}

cv::Matx33d ry(double angle)
{
    const double c = std::cos(angle), s = std::sin(angle);
    return cv::Matx33d(c, 0, s, 0, 1, 0, -s, 0, c);
}

cv::Matx33d rz(double angle)
{
    const double c = std::cos(angle), s = std::sin(angle);
    return cv::Matx33d(c, -s, 0, s, c, 0, 0, 0, 1);
}

cv::Matx33d quaternionToMatrix(const Vector3 &rotation, double w)
{
    const double x = rotation[0], y = rotation[1], z = rotation[2];
    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm < 1e-15) return cv::Matx33d::eye();
    w /= norm;
    const double nx = x / norm, ny = y / norm, nz = z / norm;
    return cv::Matx33d(1 - 2 * (ny * ny + nz * nz), 2 * (nx * ny - nz * w), 2 * (nx * nz + ny * w),
                       2 * (nx * ny + nz * w), 1 - 2 * (nx * nx + nz * nz), 2 * (ny * nz - nx * w),
                       2 * (nx * nz - ny * w), 2 * (ny * nz + nx * w), 1 - 2 * (nx * nx + ny * ny));
}

PoseConvention effectiveConvention(const PoseInputSpec &spec)
{
    if (spec.adapter == PoseAdapterKind::Kuka) return PoseConvention::KukaAbcZyx;
    if (spec.adapter == PoseAdapterKind::Fanuc) return PoseConvention::FanucWprXyz;
    if (spec.convention != PoseConvention::Generic) return spec.convention;
    switch (spec.rotationFormat) {
    case RotationFormat::EulerXYZ: return PoseConvention::EulerXYZIntrinsic;
    case RotationFormat::RPY: return PoseConvention::RpyZyx;
    case RotationFormat::Rodrigues:
    case RotationFormat::QuaternionWXYZ: return PoseConvention::Generic;
    }
    return PoseConvention::Generic;
}

Vector4 eulerXyzFromMatrix(const cv::Matx33d &matrix)
{
    const double y = std::asin(std::clamp(matrix(0, 2), -1.0, 1.0));
    const double cosY = std::cos(y);
    if (std::abs(cosY) > 1e-8) {
        return {std::atan2(-matrix(1, 2), matrix(2, 2)), y,
                std::atan2(-matrix(0, 1), matrix(0, 0)), 0.0};
    }
    const double x = matrix(0, 2) >= 0.0 ? std::atan2(matrix(1, 0), matrix(1, 1))
                                        : std::atan2(-matrix(1, 0), matrix(1, 1));
    return {x, y, 0.0, 0.0};
}

Vector4 rpyFromMatrix(const cv::Matx33d &matrix)
{
    const double pitch = std::asin(std::clamp(-matrix(2, 0), -1.0, 1.0));
    const double cosPitch = std::cos(pitch);
    if (std::abs(cosPitch) > 1e-8) {
        return {std::atan2(matrix(2, 1), matrix(2, 2)), pitch,
                std::atan2(matrix(1, 0), matrix(0, 0)), 0.0};
    }
    return {0.0, pitch, std::atan2(-matrix(0, 1), matrix(1, 1)), 0.0};
}

double angleScale(AngleUnit unit)
{
    return unit == AngleUnit::Degrees ? CV_PI / 180.0 : 1.0;
}

double lengthScale(LengthUnit unit)
{
    return unit == LengthUnit::Millimeters ? 1e-3 : 1.0;
}

} // namespace

NormalizationResult normalize(const Vector4 &rotation, const Vector3 &translation,
                              const PoseInputSpec &spec)
{
    NormalizationResult result;
    if (!matrix::isFinite(rotation) || !matrix::isFinite(translation)) {
        result.error = QStringLiteral("旋转或平移包含非有限数值。");
        return result;
    }

    const double scale = angleScale(spec.angleUnit);
    cv::Matx33d rotationMatrix;
    const PoseConvention convention = effectiveConvention(spec);
    if (convention == PoseConvention::KukaAbcZyx) {
        rotationMatrix = rz(rotation[0] * scale) * ry(rotation[1] * scale)
                        * rx(rotation[2] * scale);
    } else if (convention == PoseConvention::FanucWprXyz) {
        rotationMatrix = rz(rotation[2] * scale) * ry(rotation[1] * scale)
                        * rx(rotation[0] * scale);
    } else if (spec.rotationFormat == RotationFormat::QuaternionWXYZ) {
        const double norm = std::sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1]
                                      + rotation[2] * rotation[2] + rotation[3] * rotation[3]);
        if (norm < 1e-15) {
            result.error = QStringLiteral("四元数不能为零。");
            return result;
        }
        rotationMatrix = quaternionToMatrix({rotation[1], rotation[2], rotation[3]}, rotation[0]);
    } else if (convention == PoseConvention::EulerXYZIntrinsic) {
        rotationMatrix = rx(rotation[0] * scale) * ry(rotation[1] * scale)
                        * rz(rotation[2] * scale);
    } else if (convention == PoseConvention::RpyZyx) {
        rotationMatrix = rz(rotation[2] * scale) * ry(rotation[1] * scale)
                        * rx(rotation[0] * scale);
    } else {
        const Vector3 radians{rotation[0] * scale, rotation[1] * scale, rotation[2] * scale};
        cv::Rodrigues(cv::Vec3d(radians[0], radians[1], radians[2]), rotationMatrix);
    }

    result.rotation = matrix::toRodrigues(rotationMatrix);
    const double scaleLength = lengthScale(spec.lengthUnit);
    result.translation = {translation[0] * scaleLength, translation[1] * scaleLength,
                          translation[2] * scaleLength};
    result.success = true;
    return result;
}

NormalizationResult normalizeKukaAbc(const Vector4 &rotation, const Vector3 &translation,
                                     AngleUnit angleUnit, LengthUnit lengthUnit)
{
    PoseInputSpec spec = defaultSpec(PoseAdapterKind::Kuka);
    spec.angleUnit = angleUnit;
    spec.lengthUnit = lengthUnit;
    return normalize(rotation, translation, spec);
}

NormalizationResult normalizeFanucWpr(const Vector4 &rotation, const Vector3 &translation,
                                      AngleUnit angleUnit, LengthUnit lengthUnit)
{
    PoseInputSpec spec = defaultSpec(PoseAdapterKind::Fanuc);
    spec.angleUnit = angleUnit;
    spec.lengthUnit = lengthUnit;
    return normalize(rotation, translation, spec);
}

Vector4 rotationToFormat(const Vector3 &rodriguesRadians, RotationFormat format, AngleUnit angleUnit)
{
    cv::Matx33d matrix;
    cv::Rodrigues(cv::Vec3d(rodriguesRadians[0], rodriguesRadians[1], rodriguesRadians[2]), matrix);
    Vector4 result{};
    if (format == RotationFormat::Rodrigues) {
        result = {rodriguesRadians[0], rodriguesRadians[1], rodriguesRadians[2], 0.0};
    } else if (format == RotationFormat::QuaternionWXYZ) {
        const double trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);
        if (trace > 0.0) {
            const double s = 2.0 * std::sqrt(trace + 1.0);
            result = {0.25 * s, (matrix(2, 1) - matrix(1, 2)) / s,
                      (matrix(0, 2) - matrix(2, 0)) / s, (matrix(1, 0) - matrix(0, 1)) / s};
        } else if (matrix(0, 0) > matrix(1, 1) && matrix(0, 0) > matrix(2, 2)) {
            const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + matrix(0, 0) - matrix(1, 1) - matrix(2, 2)));
            result = {(matrix(2, 1) - matrix(1, 2)) / s, 0.25 * s,
                      (matrix(0, 1) + matrix(1, 0)) / s, (matrix(0, 2) + matrix(2, 0)) / s};
        } else if (matrix(1, 1) > matrix(2, 2)) {
            const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + matrix(1, 1) - matrix(0, 0) - matrix(2, 2)));
            result = {(matrix(0, 2) - matrix(2, 0)) / s, (matrix(0, 1) + matrix(1, 0)) / s,
                      0.25 * s, (matrix(1, 2) + matrix(2, 1)) / s};
        } else {
            const double s = 2.0 * std::sqrt(std::max(0.0, 1.0 + matrix(2, 2) - matrix(0, 0) - matrix(1, 1)));
            result = {(matrix(1, 0) - matrix(0, 1)) / s, (matrix(0, 2) + matrix(2, 0)) / s,
                      (matrix(1, 2) + matrix(2, 1)) / s, 0.25 * s};
        }
    } else if (format == RotationFormat::EulerXYZ) {
        result = eulerXyzFromMatrix(matrix);
    } else {
        result = rpyFromMatrix(matrix);
    }
    if (format != RotationFormat::QuaternionWXYZ && angleUnit == AngleUnit::Degrees)
        for (double &value : result) value *= 180.0 / CV_PI;
    return result;
}

Vector4 rotationToConvention(const Vector3 &rodriguesRadians, PoseConvention convention,
                             AngleUnit angleUnit)
{
    const Vector4 generic = rotationToFormat(rodriguesRadians,
                                             convention == PoseConvention::EulerXYZIntrinsic
                                                 ? RotationFormat::EulerXYZ
                                                 : RotationFormat::RPY,
                                             angleUnit);
    if (convention == PoseConvention::KukaAbcZyx)
        return {generic[2], generic[1], generic[0], 0.0};
    if (convention == PoseConvention::FanucWprXyz)
        return generic;
    return generic;
}

PoseInputSpec defaultSpec(PoseAdapterKind adapter)
{
    PoseInputSpec spec;
    spec.adapter = adapter;
    switch (adapter) {
    case PoseAdapterKind::Generic: break;
    case PoseAdapterKind::UniversalRobots:
        spec.rotationFormat = RotationFormat::Rodrigues;
        spec.convention = PoseConvention::Generic;
        spec.angleUnit = AngleUnit::Radians;
        spec.lengthUnit = LengthUnit::Meters;
        break;
    case PoseAdapterKind::Kuka:
        spec.rotationFormat = RotationFormat::EulerXYZ;
        spec.convention = PoseConvention::KukaAbcZyx;
        spec.angleUnit = AngleUnit::Degrees;
        spec.lengthUnit = LengthUnit::Millimeters;
        break;
    case PoseAdapterKind::Fanuc:
        spec.rotationFormat = RotationFormat::RPY;
        spec.convention = PoseConvention::FanucWprXyz;
        spec.angleUnit = AngleUnit::Degrees;
        spec.lengthUnit = LengthUnit::Millimeters;
        break;
    }
    return spec;
}

} // namespace handeye::pose
