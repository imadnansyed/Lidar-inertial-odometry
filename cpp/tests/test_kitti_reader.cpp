#include <iostream>
#include <fstream>
#include <cassert>
#include <filesystem>

#include "../include/lio/io/kitti_reader.hpp"

int main()
{
    using namespace lio;

    const auto file_path =
        std::filesystem::temp_directory_path()
        / "test_kitti.bin";

    // Generate sample KITTI data
    float data[] = {
        1.0f, 2.0f, 3.0f, 0.5f,
        4.0f, 5.0f, 6.0f, 0.8f
    };

    {
        std::ofstream file(file_path, std::ios::binary);

        file.write(
            reinterpret_cast<const char*>(data),
            sizeof(data)
        );

        assert(file.good());
    }

    // Read the scan
    KittiReader reader;

    LidarScan scan =
        reader.readVelodyneScan(file_path, 100000000);

    // Validate
    assert(scan.timestamp_ns == 100000000);
    assert(scan.points.size() == 2);

    assert(scan.points[0].xyz.isApprox(
        Eigen::Vector3d(1.0, 2.0, 3.0)
    ));

    assert(scan.points[0].intensity == 0.5f);

    assert(scan.points[1].xyz.isApprox(
        Eigen::Vector3d(4.0, 5.0, 6.0)
    ));

    assert(scan.points[1].intensity == 0.8f);

    std::filesystem::remove(file_path);

    std::cout << "All KITTI reader tests passed!\n";

    return 0;
}