#include "../include/lio/preprocessing/voxel_filter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main()
{
    lio::PointCloud input;

    // Same voxel: [0, 1) x [0, 1) x [0, 1)
    input.push_back({Eigen::Vector3d(0.1, 0.1, 0.1), 1.0f});
    input.push_back({Eigen::Vector3d(0.3, 0.3, 0.3), 3.0f});

    // Different voxel
    input.push_back({Eigen::Vector3d(1.2, 0.1, 0.1), 5.0f});

    // Negative-coordinate voxel
    input.push_back({Eigen::Vector3d(-0.1, 0.1, 0.1), 7.0f});

    lio::VoxelFilter filter(1.0);

    const lio::PointCloud output = filter.filter(input);

    assert(output.size() == 3);

    // Find centroid of the first voxel
    bool centroid_found = false;

    for (const auto& point : output) {
        if (std::abs(point.xyz.x() - 0.2) < 1e-9 &&
            std::abs(point.xyz.y() - 0.2) < 1e-9 &&
            std::abs(point.xyz.z() - 0.2) < 1e-9) {

            assert(std::abs(point.intensity - 2.0f) < 1e-6);
            centroid_found = true;
        }
    }

    assert(centroid_found);

    // Empty input
    const lio::PointCloud empty;
    assert(filter.filter(empty).empty());

    // Invalid voxel size
    bool exception_thrown = false;

    try {
        lio::VoxelFilter invalid_filter(0.0);
    }
    catch (const std::invalid_argument&) {
        exception_thrown = true;
    }

    assert(exception_thrown);

    std::cout << "All voxel filter tests passed!\n";

    return 0;
}