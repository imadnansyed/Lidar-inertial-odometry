#pragma once

#include "../registration/point_to_point_icp.hpp"

#include <Eigen/Dense>
#include <cstddef>

namespace lio {

struct RelativePoseEstimate {
    // Maps source-frame points into target-frame coordinates.
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    double rmse = 0.0;
    std::size_t iterations = 0;
    std::size_t num_correspondences = 0;

    bool converged = false;

    Eigen::Matrix4d matrix() const;

    RelativePoseEstimate inverse() const;
};

class RelativePoseEstimator {
public:
    RelativePoseEstimate estimate(
        const PointCloud& source,
        const PointCloud& target,
        double max_correspondence_distance = 2.0,
        std::size_t max_iterations = 50,
        double tolerance = 1e-6
    ) const;
};

}