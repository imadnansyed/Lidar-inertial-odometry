#include <iostream>
#include <cassert>

#include "../include/lio/common/navigationState.hpp"

int main()
{
    using namespace lio;

    NavigationState state;

    // Test 1: Default state
    assert(state.timestamp_ns == 0);

    assert(state.velocity.isApprox(
        Eigen::Vector3d::Zero()
    ));

    assert(state.gravity.isApprox(
        Eigen::Vector3d(0.0, 0.0, -9.81)
    ));

    // Test 2: Set IMU pose in world frame
    Eigen::Vector3d imu_position(1.0, 0.0, 0.0);

    state.T_WI = SE3(
        Eigen::Matrix3d::Identity(),
        imu_position
    );

    // Test 3: Set LiDAR-to-IMU extrinsic
    Eigen::Vector3d lidar_translation(0.0, 2.0, 0.0);

    state.T_IL = SE3(
        Eigen::Matrix3d::Identity(),
        lidar_translation
    );

    // Test 4: Calculate LiDAR pose in world frame
    SE3 T_WL = state.lidarPose();

    Eigen::Vector3d expected_position(1.0, 2.0, 0.0);

    assert(T_WL.translation().isApprox(
        expected_position, 1e-9
    ));

    std::cout << "All NavigationState tests passed!\n";

    return 0;
}