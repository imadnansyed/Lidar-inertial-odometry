#include "../include/lio/common/se3.hpp"

namespace lio {

// Default Constructor: identity transformation R : 3x3 identity, t : 3x1 zero

SE3::SE3() 
    : R_(Eigen::Matrix3d::Identity()),
      t_(Eigen::Vector3d::Zero())
{
}

// Constructor
SE3::SE3(
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation
)
    : R_(rotation),
      t_(translation)
{
}

// Identity Implementation
SE3 SE3::Identity()
    {
        return SE3(
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d::Zero()
        );
    }

// Access rotation
const Eigen::Matrix3d& SE3::rotation() const
{
    return R_;
}

// Access translation
const Eigen::Vector3d& SE3::translation() const
{
    return t_;
}

// Transform a point: p_A = R_AB * p_B + t_AB
Eigen::Vector3d SE3::operator*(
    const Eigen::Vector3d& point
) const
{
    return R_ * point + t_;
}

// Compose transformations
SE3 SE3::operator*(const SE3& other) const
{
    Eigen::Matrix3d R_new = R_ * other.R_;

    Eigen::Vector3d t_new =
        R_ * other.t_ + t_;

    return SE3(R_new, t_new);
}

// Inverse transformation
SE3 SE3::inverse() const
{
    Eigen::Matrix3d R_inv = R_.transpose();

    Eigen::Vector3d t_inv = -R_inv * t_;

    return SE3(R_inv, t_inv);
}

// Homogeneous transformation matrix
Eigen::Matrix4d SE3::matrix() const
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T.block<3, 3>(0, 0) = R_;
    T.block<3, 1>(0, 3) = t_;

    return T;
}

} // namespace lio