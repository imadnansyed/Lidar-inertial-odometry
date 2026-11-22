#pragma once

#include "../include/lio/mapping/local_map.hpp"

#include <Eigen/Dense>
#include <cstddef>
#include <limits>

namespace lio {

struct ScanToMapConfig {
    std::size_t max_iterations = 20;
    std::size_t min_correspondences = 10;

    double max_correspondence_distance = 1.0;
    double translation_epsilon = 1e-4;
    double rotation_epsilon = 1e-4;
};

struct ScanToMapResult {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    bool success = false;
    bool converged = false;

    std::size_t iterations = 0;
    std::size_t num_correspondences = 0;

    double fitness = std::numeric_limits<double>::infinity();
};

class ScanToMapRegistration {
public:
    ScanToMapRegistration(
        const LocalMap& map,
        const ScanToMapConfig& config = ScanToMapConfig{}
    );

    ScanToMapResult align(
        const PointCloud& scan,
        const Eigen::Matrix3d& initial_R = Eigen::Matrix3d::Identity(),
        const Eigen::Vector3d& initial_t = Eigen::Vector3d::Zero()
    ) const;

private:
    const LocalMap& map_;
    ScanToMapConfig config_;

    void validateConfig() const;
};

}