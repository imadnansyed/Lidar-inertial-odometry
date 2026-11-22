#pragma once

#include "../include/lio/common/types.hpp"
#include "correspondence.hpp"

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

namespace lio {

struct RegistrationMetrics {
    double rmse = 0.0;
    double mean_error = 0.0;
    double fitness_score = 0.0;

    std::size_t inlier_count = 0;
    std::size_t source_count = 0;

    bool valid = false;
};

class RegistrationEvaluator {
public:
    static RegistrationMetrics evaluate(
        const PointCloud& source,
        const PointCloud& target,
        const std::vector<Correspondence>& correspondences,
        const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t
    );
};

}