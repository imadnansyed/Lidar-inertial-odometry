#include "../include/lio/io/kitti_sequence.hpp"

#include "../include/lio/io/kitti_reader.hpp"
#include "../include/lio/io/timestamp_reader.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace lio {

KittiSequence::KittiSequence(
    const std::filesystem::path& sequence_path
)
    : sequence_path_(sequence_path)
{
}

void KittiSequence::load()
{
    const auto velodyne_path = sequence_path_ / "velodyne";
    const auto timestamp_path = sequence_path_ / "times.txt";

    if (!std::filesystem::exists(velodyne_path) ||
        !std::filesystem::is_directory(velodyne_path)) {
        throw std::runtime_error(
            "KITTI velodyne directory not found: " +
            velodyne_path.string()
        );
    }

    TimestampReader timestamp_reader;
    KittiReader lidar_reader;

    const auto timestamps =
        timestamp_reader.readTimestamps(timestamp_path);

    scans_.clear();
    scans_.reserve(timestamps.size());

    for (std::size_t i = 0; i < timestamps.size(); ++i) {

        std::ostringstream filename;

        filename << std::setw(6)
                 << std::setfill('0')
                 << i << ".bin";

        const auto scan_path = velodyne_path / filename.str();

        if (!std::filesystem::exists(scan_path)) {
            throw std::runtime_error(
                "Missing KITTI scan: " + scan_path.string()
            );
        }

        LidarScan scan = lidar_reader.readVelodyneScan(
            scan_path,
            timestamps[i]
        );

        scans_.push_back(std::move(scan));
    }
}

std::size_t KittiSequence::size() const
{
    return scans_.size();
}

const LidarScan& KittiSequence::getScan(
    std::size_t index
) const
{
    if (index >= scans_.size()) {
        throw std::out_of_range(
            "KITTI scan index out of range"
        );
    }

    return scans_[index];
}

const std::vector<LidarScan>& KittiSequence::scans() const
{
    return scans_;
}

} // namespace lio