#include "../include/lio/odometry/trajectory_io.hpp"

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

int main()
{
    lio::Trajectory trajectory;

    trajectory.addPose(
        1000000000,
        lio::SE3(
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(1, 2, 3)
        )
    );

    trajectory.addPose(
        2000000000,
        lio::SE3(
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(4, 5, 6)
        )
    );

    const std::string kitti_path = "test_kitti.txt";
    const std::string tum_path = "test_tum.txt";

    lio::TrajectoryWriter::writeKITTI(
        trajectory,
        kitti_path
    );

    lio::TrajectoryWriter::writeTUM(
        trajectory,
        tum_path
    );

    // Check KITTI format.
    {
        std::ifstream file(kitti_path);

        assert(file.is_open());

        std::string line;
        std::getline(file, line);

        std::stringstream stream(line);

        double value;
        int count = 0;

        while (stream >> value) {
            ++count;
        }

        assert(count == 12);
    }

    // Check TUM format.
    {
        std::ifstream file(tum_path);

        assert(file.is_open());

        std::string line;
        std::getline(file, line);

        std::stringstream stream(line);

        double value;
        int count = 0;

        while (stream >> value) {
            ++count;
        }

        assert(count == 8);
    }

    std::cout << "Trajectory export tests passed!\n";

    return 0;
}