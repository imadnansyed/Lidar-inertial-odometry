#include "../include/lio/registration/correspondence.hpp"

#include <stdexcept>

namespace lio {

CorrespondenceSearch::CorrespondenceSearch(
    const PointCloud& target
)
    : target_search_(target)
{
}

std::vector<Correspondence> CorrespondenceSearch::find(
    const PointCloud& source,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    double max_distance
) const
{
    if (!R.allFinite() || !t.allFinite()) {
        throw std::invalid_argument(
            "Transformation contains invalid values"
        );
    }

    if (max_distance < 0.0 || !std::isfinite(max_distance)) {
        throw std::invalid_argument(
            "Maximum correspondence distance must be finite and non-negative"
        );
    }

    std::vector<Correspondence> correspondences;
    correspondences.reserve(source.size());

    for (std::size_t i = 0; i < source.size(); ++i) {

        // Transform source point into target coordinate frame.
        const Eigen::Vector3d transformed =
            R * source[i].xyz + t;

        // Find nearest target point.
        auto result = target_search_.search(
            transformed,
            max_distance
        );

        if (!result.has_value()) {
            continue;
        }

        correspondences.push_back({
            i,
            result->index,
            result->squared_distance
        });
    }

    return correspondences;
}

}