#include "../include/lio/preprocessing/invalid_point_filter.hpp"

#include <cmath>

namespace lio {

PointCloud InvalidPointFilter::filter(const PointCloud& input) const
{
    PointCloud output;
    output.reserve(input.size());

    for (const auto& point : input) {

        if (!point.xyz.allFinite()) {
            continue;
        }

        if (!std::isfinite(point.intensity)) {
            continue;
        }

        output.push_back(point);
    }

    return output;
}

}