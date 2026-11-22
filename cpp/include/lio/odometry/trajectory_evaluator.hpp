#pragma once

#include "trajectory.hpp"

#include <cstddef>

namespace lio {

struct TrajectoryMetrics {
    double ate_rmse = 0.0;

    double rpe_translation_rmse = 0.0;

    double rpe_rotation_rmse_rad = 0.0;

    std::size_t num_poses = 0;

    std::size_t num_rpe_pairs = 0;

    SE3 alignment = SE3::Identity();
};

class TrajectoryEvaluator {
public:
    static TrajectoryMetrics evaluate(
        const Trajectory& estimated,
        const Trajectory& ground_truth,
        std::size_t rpe_delta = 1
    );

};

}