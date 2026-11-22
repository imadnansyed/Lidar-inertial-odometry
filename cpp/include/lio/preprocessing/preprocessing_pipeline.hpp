#pragma once

#include "../include/lio/common/types.hpp"
#include "../include/lio/common/config.hpp"
#include "invalid_point_filter.hpp"
#include "range_filter.hpp"
#include "voxel_filter.hpp"

namespace lio {

class PreprocessingPipeline {
public:
    explicit PreprocessingPipeline(const LioConfig& config);

    LidarScan process(const LidarScan& scan) const;

private:
    InvalidPointFilter invalid_filter_;
    RangeFilter range_filter_;
    VoxelFilter voxel_filter_;
};

}