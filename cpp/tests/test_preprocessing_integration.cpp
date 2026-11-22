#include "../include/lio/io/kitti_reader.hpp"
#include "../include/lio/preprocessing/preprocessing_pipeline.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>

int main()
{
    // Update this path to one of your KITTI scans
    const std::filesystem::path scan_path =
        "E:\\Workspace\\PC\\AI Engineer\\cv\\projects\\VO and SLAM\\LIO Master 01\\Lidar-inertial-odometry\\data\\000000.bin";

    lio::KittiReader reader;

    lio::LidarScan scan =
        reader.readVelodyneScan(scan_path, 123456789);

    assert(!scan.points.empty());

    const std::size_t original_count = scan.points.size();

    // Configure preprocessing
    lio::LioConfig config;

    config.lidar.min_range = 1.0;
    config.lidar.max_range = 100.0;
    config.lidar.voxel_size = 0.2;

    lio::PreprocessingPipeline pipeline(config);

    const lio::LidarScan processed =
        pipeline.process(scan);

    // Basic validation
    assert(processed.timestamp_ns == scan.timestamp_ns);
    assert(!processed.points.empty());

    // Downsampling should not increase point count
    assert(processed.points.size() <= original_count);

    // Verify every output point
    for (const auto& point : processed.points) {
        assert(point.xyz.allFinite());
        assert(std::isfinite(point.intensity));

        const double range = point.xyz.norm();

        assert(range >= config.lidar.min_range);
        assert(range <= config.lidar.max_range);
    }

    std::cout << "Original points: "
              << original_count << '\n';

    std::cout << "Processed points: "
              << processed.points.size() << '\n';

    std::cout << "Reduction: "
              << (1.0 -
                  static_cast<double>(processed.points.size()) /
                  original_count) * 100.0
              << "%\n";

    std::cout << "Preprocessing integration test passed!\n";

    return 0;
}