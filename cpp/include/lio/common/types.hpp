
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lio {



// 1. LiDAR Point



struct PointXYZI {
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    float intensity = 0.0f;
};




// 2. Point Cloud



using PointCloud = std::vector<PointXYZI>;




// 3. LiDAR Scan



struct LidarScan {
    int64_t timestamp_ns = 0;

    PointCloud points;
};




// 4. IMU Measurement



struct ImuSample {
    int64_t timestamp_ns = 0;

    Eigen::Vector3d acceleration =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d angular_velocity =
        Eigen::Vector3d::Zero();
};




// 5. Synchronized Measurement Group


struct MeasurementGroup {
    LidarScan lidar;

    std::vector<ImuSample> imu_measurements;
};


// 6. Estimated Pose Record


struct PoseRecord {
    int64_t timestamp_ns = 0;

    Eigen::Matrix3d rotation =
        Eigen::Matrix3d::Identity();

    Eigen::Vector3d position =
        Eigen::Vector3d::Zero();
};

}
