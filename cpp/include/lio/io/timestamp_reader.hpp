#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

namespace lio {

class TimestampReader {
public:
    std::vector<int64_t> readTimestamps(
        const std::filesystem::path& file_path
    ) const;

private:
    int64_t parseTimestamp(const std::string& timestamp) const;
};

} // namespace lio