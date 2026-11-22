#include "../include/lio/preprocessing/voxel_filter.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

namespace lio {

namespace {

struct VoxelKey {
    int64_t x;
    int64_t y;
    int64_t z;

    bool operator==(const VoxelKey& other) const {
        return x == other.x &&
               y == other.y &&
               z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        std::size_t h1 = std::hash<int64_t>{}(key.x);
        std::size_t h2 = std::hash<int64_t>{}(key.y);
        std::size_t h3 = std::hash<int64_t>{}(key.z);

        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct VoxelAccumulator {
    Eigen::Vector3d position_sum = Eigen::Vector3d::Zero();
    double intensity_sum = 0.0;
    std::size_t count = 0;
};

}

VoxelFilter::VoxelFilter(double voxel_size)
    : voxel_size_(voxel_size)
{
    if (!std::isfinite(voxel_size_) || voxel_size_ <= 0.0) {
        throw std::invalid_argument("Voxel size must be positive");
    }
}

PointCloud VoxelFilter::filter(const PointCloud& input) const
{
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;

    for (const auto& point : input) {
        if (!point.xyz.allFinite() ||
            !std::isfinite(point.intensity)) {
            continue;
        }

        VoxelKey key{
            static_cast<int64_t>(std::floor(point.xyz.x() / voxel_size_)),
            static_cast<int64_t>(std::floor(point.xyz.y() / voxel_size_)),
            static_cast<int64_t>(std::floor(point.xyz.z() / voxel_size_))
        };

        auto& voxel = voxels[key];

        voxel.position_sum += point.xyz;
        voxel.intensity_sum += point.intensity;
        voxel.count++;
    }

    PointCloud output;
    output.reserve(voxels.size());

    for (const auto& entry : voxels) {
        const auto& voxel = entry.second;

        const double count = static_cast<double>(voxel.count);

        output.push_back({
            voxel.position_sum / count,
            static_cast<float>(voxel.intensity_sum / count)
        });
    }

    return output;
}

}