#include "../include/lio/odometry/trajectory_evaluator_comparison.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    lio::Trajectory cpp_trajectory;
    lio::Trajectory python_trajectory;

    for (int i = 0; i < 5; ++i) {
        const std::int64_t timestamp =
            (i + 1) * 1000000000LL;

        const Eigen::Vector3d position(
            static_cast<double>(i),
            static_cast<double>(i * 2),
            0.0
        );

        const lio::SE3 pose(
            Eigen::Matrix3d::Identity(),
            position
        );

        cpp_trajectory.addPose(timestamp, pose);
        python_trajectory.addPose(timestamp, pose);
    }

    const auto result =
        lio::TrajectoryComparator::compare(
            cpp_trajectory,
            python_trajectory
        );

    assert(result.num_poses == 5);

    assert(result.translation_rmse < 1e-10);
    assert(result.rotation_rmse_rad < 1e-10);

    assert(result.translation_max_error < 1e-10);
    assert(result.rotation_max_error_rad < 1e-10);

    std::cout << "Translation RMSE: "
              << result.translation_rmse << '\n';

    std::cout << "Rotation RMSE: "
              << result.rotation_rmse_rad << '\n';

    std::cout << "Trajectory comparison tests passed!\n";

    return 0;
}