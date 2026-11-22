#pragma once

#include "../include/lio/common/types.hpp"

namespace lio {

class VoxelFilter {
public:
    explicit VoxelFilter(double voxel_size);

    PointCloud filter(const PointCloud& input) const;

private:
    double voxel_size_;
};

}