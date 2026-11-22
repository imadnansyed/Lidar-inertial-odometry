#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lio {

class SE3 {
public:
    // Default constructor: identity transformation
    SE3();

    // Construct from rotation and translation
    SE3(
        const Eigen::Matrix3d& rotation,
        const Eigen::Vector3d& translation
    );

    // Create identity transformation
    static SE3 Identity();

    // Access rotation and translation
    const Eigen::Matrix3d& rotation() const;
    const Eigen::Vector3d& translation() const;

    // Transform a point
    Eigen::Vector3d operator*(
        const Eigen::Vector3d& point
    ) const;

    // Compose two transformations
    SE3 operator*(const SE3& other) const;

    // Inverse transformation
    SE3 inverse() const;

    // Return homogeneous 4x4 matrix
    Eigen::Matrix4d matrix() const;

private:
    Eigen::Matrix3d R_;
    Eigen::Vector3d t_;
};

} // namespace lio