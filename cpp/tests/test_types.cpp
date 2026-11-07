
#include <iostream>
#include "E:\Workspace\PC\AI Engineer\cv\projects\VO and SLAM\LIO Master 01\Lidar-inertial-odometry\cpp\include\lio\common\types.hpp"

int main() {

    // Create a LiDAR point
    lio::PointXYZI point;
    point.xyz = Eigen::Vector3d(1.0, 2.0, 3.0);
    point.intensity = 0.8f;

    // Create a point cloud
    lio::PointCloud cloud;
    cloud.push_back(point);

    // Create a LiDAR scan
    lio::LidarScan scan;
    scan.timestamp_ns = 100000000;
    scan.points = cloud;

    // Create an IMU sample
    lio::ImuSample imu;
    imu.timestamp_ns = 99000000;
    imu.acceleration = Eigen::Vector3d(0.1, 0.2, 9.7);
    imu.angular_velocity = Eigen::Vector3d(0.01, 0.02, 0.03);

    // Create a measurement group
    lio::MeasurementGroup group;
    group.lidar = scan;
    group.imu_measurements.push_back(imu);

    // Create an estimated pose
    lio::PoseRecord pose;
    pose.timestamp_ns = scan.timestamp_ns;

    std::cout << "LiDAR points: "
              << group.lidar.points.size() << '\n';

    std::cout << "IMU measurements: "
              << group.imu_measurements.size() << '\n';

    std::cout << "LiDAR timestamp: "
              << group.lidar.timestamp_ns << '\n';

    std::cout << "Pose position: "
              << pose.position.transpose() << '\n';

    return 0;
}
