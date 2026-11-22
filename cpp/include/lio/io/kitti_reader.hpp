#pragma once

#include <cstdint>
#include <filesystem>

#include "../include/lio/common/types.hpp"

namespace lio {

class KittiReader {
public:
    LidarScan readVelodyneScan(
        const std::filesystem::path& file_path,
        int64_t timestamp_ns = 0
    ) const;
};

} // namespace lio