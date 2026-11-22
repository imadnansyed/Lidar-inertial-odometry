#pragma once
#include <Eigen/Geometry>

namespace lio {
namespace math {

// Convert a 3D vector into a skew-symmetric matrix
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// SO(3) exponential map: rotation vector -> rotation matrix
Eigen::Matrix3d Exp(const Eigen::Vector3d& omega);

// SO(3) logarithm map: rotation matrix -> rotation vector
Eigen::Vector3d Log(const Eigen::Matrix3d& R);

// Project a matrix onto the nearest valid rotation matrix
Eigen::Matrix3d normalizeRotation(const Eigen::Matrix3d& R);

} // namespace math
} // namespace lio