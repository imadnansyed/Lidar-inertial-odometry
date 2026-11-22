#include "../include/lio/odometry/pose_accumulator.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    lio::PoseAccumulator accumulator;

    // Two known relative transformations.
    lio::RelativePoseEstimate relative;

    relative.R = Eigen::Matrix3d::Identity();
    relative.t = Eigen::Vector3d(-1.0, 0.0, 0.0);
    relative.converged = true;

    // ICP source -> target is -1 in X.
    // Its inverse moves the accumulated pose +1 in X.
    auto pose1 = accumulator.update(relative);

    assert(std::abs(pose1.translation().x() - 1.0) < 1e-8);

    relative.t = Eigen::Vector3d(0.0, -2.0, 0.0);

    auto pose2 = accumulator.update(relative);

    assert(std::abs(pose2.translation().x() - 1.0) < 1e-8);
    assert(std::abs(pose2.translation().y() - 2.0) < 1e-8);

    assert(accumulator.size() == 3);

    // Check reset.
    accumulator.reset();

    assert(accumulator.size() == 1);
    assert(
        accumulator.currentPose().matrix().isApprox(
            Eigen::Matrix4d::Identity()
        )
    );

    std::cout << "Pose accumulation tests passed!\n";

    return 0;
}