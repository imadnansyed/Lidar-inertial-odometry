#include "../include/lio/odometry/trajectory_io.hpp"

#include <Eigen/Geometry>

#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace lio {

void TrajectoryWriter::writeKITTI(
    const Trajectory& trajectory,
    const std::string& filepath
)
{
    std::ofstream file(filepath);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Unable to open KITTI trajectory output file"
        );
    }

    file << std::fixed << std::setprecision(9);

    for (const auto& record : trajectory.poses()) {
        const Eigen::Matrix4d T = record.pose.matrix();

        // KITTI expects 3 rows x 4 columns.
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {
                file << T(row, col);

                if (!(row == 2 && col == 3)) {
                    file << " ";
                }
            }
        }

        file << "\n";
    }

    if (!file.good()) {
        throw std::runtime_error(
            "Failed while writing KITTI trajectory"
        );
    }
}

void TrajectoryWriter::writeTUM(
    const Trajectory& trajectory,
    const std::string& filepath
)
{
    std::ofstream file(filepath);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Unable to open TUM trajectory output file"
        );
    }

    file << std::fixed << std::setprecision(9);

    for (const auto& record : trajectory.poses()) {
        const double timestamp =
            static_cast<double>(record.timestamp_ns) * 1e-9;

        const Eigen::Vector3d position =
            record.pose.translation();

        Eigen::Quaterniond quaternion(
            record.pose.rotation()
        );

        quaternion.normalize();

        file
            << timestamp << " "
            << position.x() << " "
            << position.y() << " "
            << position.z() << " "
            << quaternion.x() << " "
            << quaternion.y() << " "
            << quaternion.z() << " "
            << quaternion.w()
            << "\n";
    }

    if (!file.good()) {
        throw std::runtime_error(
            "Failed while writing TUM trajectory"
        );
    }
}

}