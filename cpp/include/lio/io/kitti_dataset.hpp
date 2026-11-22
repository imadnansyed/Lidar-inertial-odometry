#pragma once

#include "../include/lio/io/dataset.hpp"
#include "../include/lio/io/kitti_sequence.hpp"

#include <cstddef>
#include <filesystem>

namespace lio {

class KittiDataset : public Dataset {
public:
    explicit KittiDataset(const std::filesystem::path& sequence_path);

    void load();

    bool hasNext() const override;
    LidarScan next() override;
    void reset() override;
    std::size_t size() const override;

private:
    KittiSequence sequence_;
    std::size_t current_index_ = 0;
};

}