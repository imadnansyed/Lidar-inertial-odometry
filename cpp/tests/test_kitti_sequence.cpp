#include <iostream>
#include <fstream>
#include <cassert>
#include <filesystem>

#include "../include/lio/io/kitti_sequence.hpp"

int main()
{
    using namespace lio;

    const auto root =
        std::filesystem::temp_directory_path()
        / "test_kitti_sequence";

    const auto velodyne = root / "velodyne";

    std::filesystem::create_directories(velodyne);

    // Create timestamps
    {
        std::ofstream file(root / "times.txt");

        file << "2011-09-26 13:02:45.000000000\n";
        file << "2011-09-26 13:02:45.100000000\n";
    }

    // Create two dummy scans
    float scan_data[] = {
        1.0f, 2.0f, 3.0f, 0.5f
    };

    for (int i = 0; i < 2; ++i) {

        std::string filename =
            (i == 0) ? "000000.bin" : "000001.bin";

        std::ofstream file(
            velodyne / filename,
            std::ios::binary
        );

        file.write(
            reinterpret_cast<const char*>(scan_data),
            sizeof(scan_data)
        );
    }

    // Load sequence
    KittiSequence sequence(root);

    sequence.load();

    assert(sequence.size() == 2);

    assert(sequence.getScan(0).points.size() == 1);
    assert(sequence.getScan(1).points.size() == 1);

    assert(
        sequence.getScan(1).timestamp_ns -
        sequence.getScan(0).timestamp_ns
        == 100000000LL
    );

    std::filesystem::remove_all(root);

    std::cout << "All KITTI sequence tests passed!\n";

    return 0;
}