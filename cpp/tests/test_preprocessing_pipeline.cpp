#include "../include/lio/preprocessing/preprocessing_pipeline.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
    lio::LioConfig config;

    config.lidar.min_range = 1.0;
    config.lidar.max_range = 10.0;
    config.lidar.voxel_size = 1.0;

    lio::PreprocessingPipeline pipeline(config);

    lio::LidarScan scan;
    scan.timestamp_ns = 123456789;

    // Two points in the same voxel
    scan.points.push_back({
        Eigen::Vector3d(2.1, 2.1, 2.1), 1.0f
    });

    scan.points.push_back({
        Eigen::Vector3d(2.3, 2.3, 2.3), 3.0f
    });

    // Different voxel
    scan.points.push_back({
        Eigen::Vector3d(4.1, 2.1, 2.1), 5.0f
    });

    // Below minimum range
    scan.points.push_back({
        Eigen::Vector3d(0.5, 0, 0), 1.0f
    });

    // Above maximum range
    scan.points.push_back({
        Eigen::Vector3d(11, 0, 0), 1.0f
    });

    // Invalid point
    scan.points.push_back({
        Eigen::Vector3d(
            std::numeric_limits<double>::quiet_NaN(),
            0, 0
        ),
        1.0f
    });

    const lio::LidarScan output =
        pipeline.process(scan);

    // Timestamp must be preserved
    assert(output.timestamp_ns == scan.timestamp_ns);

    // Two occupied voxels
    assert(output.points.size() == 2);

    // Verify centroid
    bool centroid_found = false;

    for (const auto& point : output.points) {
        if (std::abs(point.xyz.x() - 2.2) < 1e-9) {
            assert(std::abs(point.intensity - 2.0f) < 1e-6);
            centroid_found = true;
        }
    }

    assert(centroid_found);

    std::cout << "All preprocessing pipeline tests passed!\n";

    return 0;
}