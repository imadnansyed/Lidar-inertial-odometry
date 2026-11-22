#pragma once

#include <cstddef>
#include <filesystem>
#include <vector>

#include "../include/lio/common/types.hpp"

namespace lio {

class KittiSequence {
public:
    explicit KittiSequence(
        const std::filesystem::path& sequence_path
    );

    void load();

    std::size_t size() const;

    const LidarScan& getScan(std::size_t index) const;

    const std::vector<LidarScan>& scans() const;

private:
    std::filesystem::path sequence_path_;
    std::vector<LidarScan> scans_;
};

} // namespace lio