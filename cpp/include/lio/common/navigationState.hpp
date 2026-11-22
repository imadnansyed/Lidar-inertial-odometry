#pragma once

#include <cstdint>

#include <Eigen/Core>

#include "lio/common/se3.hpp"

namespace lio {

struct NavigationState
{
    // State timestamp
    int64_t timestamp_ns = 0;

    // IMU pose in world frame: T_WI
    SE3 T_WI;

    // IMU velocity in world frame
    Eigen::Vector3d velocity =
        Eigen::Vector3d::Zero();

    // IMU measurement biases
    Eigen::Vector3d gyro_bias =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d accel_bias =
        Eigen::Vector3d::Zero();

    // Gravity expressed in world frame
    Eigen::Vector3d gravity =
        Eigen::Vector3d(0.0, 0.0, -9.81);

    // LiDAR-to-IMU extrinsic transformation: T_IL
    SE3 T_IL;

    // Calculate LiDAR pose in world frame
    SE3 lidarPose() const
    {
        return T_WI * T_IL;
    }
};

} // namespace lio