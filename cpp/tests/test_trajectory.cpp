#include "../include/lio/odometry/trajectory.hpp"

#include <cassert>
#include <iostream>
#include <stdexcept>

int main()
{
    lio::Trajectory trajectory;

    assert(trajectory.empty());

    lio::SE3 pose1(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(1, 0, 0)
    );

    lio::SE3 pose2(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(2, 0, 0)
    );

    trajectory.addPose(1000000000, pose1);
    trajectory.addPose(2000000000, pose2);

    assert(trajectory.size() == 2);

    assert(trajectory.at(0).timestamp_ns == 1000000000);
    assert(trajectory.at(1).timestamp_ns == 2000000000);

    assert(
        trajectory.at(0).pose.translation().isApprox(
            Eigen::Vector3d(1, 0, 0)
        )
    );

    // Timestamp ordering validation.
    bool exception_thrown = false;

    try {
        trajectory.addPose(1500000000, pose1);
    }
    catch (const std::invalid_argument&) {
        exception_thrown = true;
    }

    assert(exception_thrown);

    trajectory.clear();

    assert(trajectory.empty());

    std::cout << "Trajectory storage tests passed!\n";

    return 0;
}