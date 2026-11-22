#pragma once

#include "nearest_neighbor.hpp"

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

namespace lio {

struct PointToPlaneResult {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    double rmse = 0.0;
    std::size_t iterations = 0;
    std::size_t num_correspondences = 0;
    bool converged = false;
};

class PointToPlaneICP {
public:
    PointToPlaneResult align(
        const PointCloud& source,
        const PointCloud& target,
        double max_correspondence_distance,
        std::size_t max_iterations = 30,
        double tolerance = 1e-6,
        std::size_t normal_neighbors = 10
    ) const;
};

}