#include <iostream>
#include <fstream>
#include <cassert>
#include <filesystem>

#include "../include/lio/io/timestamp_reader.hpp"

int main()
{
    using namespace lio;

    const auto file_path =
        std::filesystem::temp_directory_path()
        / "test_timestamps.txt";

    // Generate sample timestamp file
    {
        std::ofstream file(file_path);

        file << "2011-09-26 13:02:45.000000000\n";
        file << "2011-09-26 13:02:45.100000000\n";
        file << "2011-09-26 13:02:45.200000000\n";
    }

    TimestampReader reader;

    const auto timestamps =
        reader.readTimestamps(file_path);

    assert(timestamps.size() == 3);

    // Each scan should be separated by 100 ms
    assert(timestamps[1] - timestamps[0] == 100000000LL);
    assert(timestamps[2] - timestamps[1] == 100000000LL);

    std::filesystem::remove(file_path);

    std::cout << "All timestamp reader tests passed!\n";

    return 0;
}