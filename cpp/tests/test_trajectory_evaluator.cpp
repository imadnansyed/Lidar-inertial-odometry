#include "../include/lio/odometry/trajectory_evaluator.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    lio::Trajectory estimated;
    lio::Trajectory ground_truth;

    // Ground-truth trajectory.
    for (int i = 0; i < 5; ++i) {
        const Eigen::Vector3d position(
            static_cast<double>(i),
            static_cast<double>(i * i),
            0.0
        );

        const lio::SE3 pose(
            Eigen::Matrix3d::Identity(),
            position
        );

        ground_truth.addPose(
            (i + 1) * 1000000000LL,
            pose
        );

        // Estimated trajectory has a constant global offset.
        const lio::SE3 estimated_pose(
            Eigen::Matrix3d::Identity(),
            position + Eigen::Vector3d(10, 5, 2)
        );

        estimated.addPose(
            (i + 1) * 1000000000LL,
            estimated_pose
        );
    }

    const auto metrics =
        lio::TrajectoryEvaluator::evaluate(
            estimated,
            ground_truth,
            1
        );

    assert(metrics.num_poses == 5);
    assert(metrics.num_rpe_pairs == 4);

    // Alignment should remove the constant offset.
    assert(metrics.ate_rmse < 1e-8);

    assert(metrics.rpe_translation_rmse < 1e-8);

    assert(metrics.rpe_rotation_rmse_rad < 1e-8);

    std::cout << "ATE RMSE: "
              << metrics.ate_rmse << '\n';

    std::cout << "RPE translation RMSE: "
              << metrics.rpe_translation_rmse << '\n';

    std::cout << "RPE rotation RMSE (rad): "
              << metrics.rpe_rotation_rmse_rad << '\n';

    std::cout << "Trajectory evaluator tests passed!\n";

    return 0;
}