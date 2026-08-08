#include "core/matrix_utils.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>

namespace handeye::matrix {

cv::Matx44d fromRodrigues(const Vector3 &rotation, const Vector3 &translation)
{
    const cv::Vec3d r(rotation[0], rotation[1], rotation[2]);
    cv::Matx33d rotationMatrix;
    cv::Rodrigues(r, rotationMatrix);
    cv::Matx44d result = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            result(row, col) = rotationMatrix(row, col);
    result(0, 3) = translation[0];
    result(1, 3) = translation[1];
    result(2, 3) = translation[2];
    return result;
}

Vector3 toRodrigues(const cv::Matx33d &rotation)
{
    cv::Vec3d vector;
    cv::Rodrigues(rotation, vector);
    return {vector[0], vector[1], vector[2]};
}

Matrix4 toArray(const cv::Matx44d &matrix)
{
    Matrix4 result{};
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            result[row][col] = matrix(row, col);
    return result;
}

cv::Matx44d toMat(const Matrix4 &matrix)
{
    cv::Matx44d result;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            result(row, col) = matrix[row][col];
    return result;
}

cv::Matx44d inverse(const cv::Matx44d &matrix)
{
    cv::Matx33d rotation;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            rotation(row, col) = matrix(row, col);
    const cv::Vec3d translation(matrix(0, 3), matrix(1, 3), matrix(2, 3));
    const cv::Matx33d transposed = rotation.t();
    const cv::Vec3d inverseTranslation = -(transposed * translation);
    cv::Matx44d result = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col)
            result(row, col) = transposed(row, col);
        result(row, 3) = inverseTranslation[row];
    }
    return result;
}

double rotationAngleDeg(const cv::Matx33d &rotation)
{
    const double trace = std::clamp((rotation(0, 0) + rotation(1, 1) + rotation(2, 2) - 1.0) / 2.0,
                                    -1.0, 1.0);
    return std::acos(trace) * 180.0 / CV_PI;
}

bool isFinite(const Vector3 &value)
{
    return std::all_of(value.begin(), value.end(), [](double item) { return std::isfinite(item); });
}

} // namespace handeye::matrix
