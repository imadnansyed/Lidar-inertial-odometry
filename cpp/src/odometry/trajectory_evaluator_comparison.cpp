#include "../include/lio/odometry/trajectory_evaluator_comparison.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lio {

namespace {

double rotationError(const Eigen::Matrix3d& R)
{
    const double cosine = std::clamp(
        (R.trace() - 1.0) * 0.5,
        -1.0,
        1.0
    );

    return std::acos(cosine);
}

}

TrajectoryComparisonResult TrajectoryComparator::compare(
    const Trajectory& cpp_trajectory,
    const Trajectory& python_trajectory
)
{
    if (cpp_trajectory.size() != python_trajectory.size()) {
        throw std::invalid_argument(
            "Trajectories must contain the same number of poses"
        );
    }

    if (cpp_trajectory.empty()) {
        throw std::invalid_argument(
            "Cannot compare empty trajectories"
        );
    }

    double translation_squared_sum = 0.0;
    double rotation_squared_sum = 0.0;

    TrajectoryComparisonResult result;

    result.num_poses = cpp_trajectory.size();

    for (std::size_t i = 0; i < cpp_trajectory.size(); ++i) {
        const auto& cpp_record = cpp_trajectory.at(i);
        const auto& python_record = python_trajectory.at(i);

        if (cpp_record.timestamp_ns != python_record.timestamp_ns) {
            throw std::invalid_argument(
                "Trajectory timestamps do not match"
            );
        }

        const SE3 error =
            python_record.pose.inverse() * cpp_record.pose;

        const double translation_error =
            error.translation().norm();

        const double rotation_error =
            rotationError(error.rotation());

        translation_squared_sum +=
            translation_error * translation_error;

        rotation_squared_sum +=
            rotation_error * rotation_error;

        result.translation_max_error = std::max(
            result.translation_max_error,
            translation_error
        );

        result.rotation_max_error_rad = std::max(
            result.rotation_max_error_rad,
            rotation_error
        );
    }

    const double N = static_cast<double>(result.num_poses);

    result.translation_rmse =
        std::sqrt(translation_squared_sum / N);

    result.rotation_rmse_rad =
        std::sqrt(rotation_squared_sum / N);

    return result;
}

}