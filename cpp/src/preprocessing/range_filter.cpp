#include "../include/lio/preprocessing/range_filter.hpp"

#include <cmath>
#include <stdexcept>

namespace lio {

RangeFilter::RangeFilter(const LidarConfig& config)
{
    if (config.min_range < 0.0 ||
        config.max_range <= config.min_range) {
        throw std::invalid_argument("Invalid range filter configuration");
    }

    min_range_squared_ = config.min_range * config.min_range;
    max_range_squared_ = config.max_range * config.max_range;
}

PointCloud RangeFilter::filter(const PointCloud& input) const
{
    PointCloud output;
    output.reserve(input.size());

    for (const auto& point : input) {
        const auto& xyz = point.xyz;

        if (!xyz.allFinite()) {
            continue;
        }

        const double distance_squared = xyz.squaredNorm();

        if (distance_squared < min_range_squared_ ||
            distance_squared > max_range_squared_) {
            continue;
        }

        output.push_back(point);
    }

    return output;
}

}