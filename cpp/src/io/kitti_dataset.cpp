#include "../include/lio/io/kitti_dataset.hpp"

#include <stdexcept>

namespace lio {

KittiDataset::KittiDataset(
    const std::filesystem::path& sequence_path
)
    : sequence_(sequence_path)
{
}

void KittiDataset::load()
{
    sequence_.load();
    current_index_ = 0;
}

bool KittiDataset::hasNext() const
{
    return current_index_ < sequence_.size();
}

LidarScan KittiDataset::next()
{
    if (!hasNext()) {
        throw std::out_of_range("No more KITTI scans available");
    }

    return sequence_.getScan(current_index_++);
}

void KittiDataset::reset()
{
    current_index_ = 0;
}

std::size_t KittiDataset::size() const
{
    return sequence_.size();
}

}