#pragma once

#include "../include/lio/common/types.hpp"
#include "../include/lio/common/config.hpp"

namespace lio {

class RangeFilter {
public:
    explicit RangeFilter(const LidarConfig& config);

    PointCloud filter(const PointCloud& input) const;

private:
    double min_range_squared_;
    double max_range_squared_;
};

}