#include "../include/lio/io/timestamp_reader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <ctime>

#ifdef _WIN32
#include <cstdlib>
#endif

namespace lio {

int64_t TimestampReader::parseTimestamp(
    const std::string& timestamp
) const
{
    std::tm time_info = {};
    std::string fractional_seconds;

    std::istringstream stream(timestamp);

    stream >> std::get_time(
        &time_info,
        "%Y-%m-%d %H:%M:%S"
    );

    if (stream.fail()) {
        throw std::runtime_error(
            "Invalid KITTI timestamp: " + timestamp
        );
    }

    stream >> fractional_seconds;

    if (fractional_seconds.size() != 10 ||
        fractional_seconds[0] != '.') {
        throw std::runtime_error(
            "Invalid timestamp fractional part: " + timestamp
        );
    }

    // Convert calendar time to Unix time.
    // KITTI timestamps are UTC.
    #ifdef _WIN32
    const std::time_t seconds = _mkgmtime(&time_info);
    #else
        const std::time_t seconds = timegm(&time_info);
    #endif

    if (seconds == static_cast<std::time_t>(-1)) {
        throw std::runtime_error(
            "Failed to convert KITTI timestamp"
        );
    }

    const int64_t nanoseconds =
        std::stoll(fractional_seconds.substr(1));

    return static_cast<int64_t>(seconds) * 1000000000LL // Tns = Ts * 10^9 + fractional part
           + nanoseconds;
}

std::vector<int64_t> TimestampReader::readTimestamps(
    const std::filesystem::path& file_path
) const
{
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open timestamp file: " + file_path.string()
        );
    }

    std::vector<int64_t> timestamps;
    std::string line;

    while (std::getline(file, line)) {

        if (line.empty()) {
            continue;
        }

        timestamps.push_back(parseTimestamp(line));
    }

    if (timestamps.empty()) {
        throw std::runtime_error(
            "Timestamp file contains no timestamps"
        );
    }

    return timestamps;
}

} // namespace lio