#include "../include/lio/common/math.hpp"

#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace lio {
namespace math {

// Skew-symmetric matrix
Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d M;

    M <<  0.0,  -v.z(),  v.y(),
          v.z(),  0.0,  -v.x(),
         -v.y(),  v.x(),  0.0;

    return M;
}

// Exponential map: so(3) -> SO(3)
Eigen::Matrix3d Exp(const Eigen::Vector3d& omega)
{
    const double theta = omega.norm();

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d W = skew(omega);
    const Eigen::Matrix3d W2 = W * W;

    // Small-angle approximation
    if (theta < 1e-8)
    {
        return I + W + 0.5 * W2;
    }

    const double A = std::sin(theta) / theta;
    const double B = (1.0 - std::cos(theta)) / (theta * theta);

    // Rodrigues' rotation formula
    return I + A * W + B * W2;
}

// Logarithm map: SO(3) -> so(3)
Eigen::Vector3d Log(const Eigen::Matrix3d& R)
{
    Eigen::AngleAxisd angle_axis(R);

    const double theta = angle_axis.angle();

    if (theta < 1e-8)
    {
        return Eigen::Vector3d::Zero();
    }

    return angle_axis.axis() * theta;
}

// Project an approximate rotation matrix onto SO(3)
Eigen::Matrix3d normalizeRotation(const Eigen::Matrix3d& R)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        R,
        Eigen::ComputeFullU | Eigen::ComputeFullV
    );

    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();

    // Ensure determinant is +1, not -1
    D(2, 2) = (U * V.transpose()).determinant();

    return U * D * V.transpose();
}

} // namespace math
} // namespace lio