#pragma once

#include "trajectory.hpp"

#include <cstddef>

namespace lio {

struct TrajectoryComparisonResult {
    double translation_rmse = 0.0;

    double rotation_rmse_rad = 0.0;

    double translation_max_error = 0.0;

    double rotation_max_error_rad = 0.0;

    std::size_t num_poses = 0;
};

class TrajectoryComparator {
public:
    static TrajectoryComparisonResult compare(
        const Trajectory& cpp_trajectory,
        const Trajectory& python_trajectory
    );
};

}