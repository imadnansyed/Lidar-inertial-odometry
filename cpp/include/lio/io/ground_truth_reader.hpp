#pragma once

#include <filesystem>
#include <vector>

#include "../include/lio/common/types.hpp"

namespace lio {

class GroundTruthReader {
public:
    std::vector<PoseRecord> read(
        const std::filesystem::path& file_path
    ) const;
};

} // namespace lio