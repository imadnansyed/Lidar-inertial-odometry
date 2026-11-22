#include <iostream>
#include <fstream>
#include <cassert>
#include <filesystem>

#include "../include/lio/io/ground_truth_reader.hpp"

int main()
{
    using namespace lio;

    const auto file_path =
        std::filesystem::temp_directory_path()
        / "test_ground_truth.txt";

    {
        std::ofstream file(file_path);

        file << "1 0 0 1  0 1 0 2  0 0 1 3\n";
        file << "0 -1 0 4  1 0 0 5  0 0 1 6\n";
    }

    GroundTruthReader reader;

    const auto poses = reader.read(file_path);

    assert(poses.size() == 2);

    // Test first pose
    assert(poses[0].position.isApprox(
        Eigen::Vector3d(1, 2, 3)
    ));

    assert(poses[0].rotation.isApprox(
        Eigen::Matrix3d::Identity()
    ));

    // Test second pose
    assert(poses[1].position.isApprox(
        Eigen::Vector3d(4, 5, 6)
    ));

    Eigen::Matrix3d expected_rotation;
    expected_rotation << 0, -1, 0,
                         1,  0, 0,
                         0,  0, 1;

    assert(poses[1].rotation.isApprox(expected_rotation));

    std::filesystem::remove(file_path);

    std::cout << "All ground-truth reader tests passed!\n";

    return 0;
}