#pragma once

#include "domain/calibration_types.h"

#include <opencv2/core.hpp>

namespace handeye::matrix {

cv::Matx44d fromRodrigues(const Vector3 &rotation, const Vector3 &translation);
Vector3 toRodrigues(const cv::Matx33d &rotation);
Matrix4 toArray(const cv::Matx44d &matrix);
cv::Matx44d toMat(const Matrix4 &matrix);
cv::Matx44d inverse(const cv::Matx44d &matrix);
double rotationAngleDeg(const cv::Matx33d &rotation);
bool isFinite(const Vector3 &value);
bool isFinite(const Vector4 &value);

} // namespace handeye::matrix
