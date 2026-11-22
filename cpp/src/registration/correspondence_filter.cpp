#include "../include/lio/registration/correspondence_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lio {

namespace {

double median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }

    const std::size_t middle = values.size() / 2;

    std::nth_element(
        values.begin(),
        values.begin() + middle,
        values.end()
    );

    const double upper = values[middle];

    if (values.size() % 2 != 0) {
        return upper;
    }

    const double lower = *std::max_element(
        values.begin(),
        values.begin() + middle
    );

    return 0.5 * (lower + upper);
}

}

std::vector<Correspondence> CorrespondenceFilter::filter(
    const std::vector<Correspondence>& correspondences,
    double max_distance,
    double mad_scale
)
{
    if (!std::isfinite(max_distance) || max_distance <= 0.0 ||
        !std::isfinite(mad_scale) || mad_scale <= 0.0) {
        throw std::invalid_argument("Invalid correspondence filter settings");
    }

    if (correspondences.empty()) {
        return {};
    }

    const double max_squared_distance =
        max_distance * max_distance;

    // First pass: apply the absolute distance limit.
    std::vector<Correspondence> candidates;
    std::vector<double> distances;

    for (const auto& c : correspondences) {
        if (!std::isfinite(c.squared_distance) ||
            c.squared_distance < 0.0 ||
            c.squared_distance > max_squared_distance) {
            continue;
        }

        candidates.push_back(c);
        distances.push_back(c.squared_distance);
    }

    if (candidates.empty()) {
        return {};
    }

    // Calculate median and MAD.
    const double center = median(distances);

    std::vector<double> deviations;
    deviations.reserve(distances.size());

    for (double distance : distances) {
        deviations.push_back(std::abs(distance - center));
    }

    const double mad = median(deviations);

    // Convert MAD to a robust standard-deviation estimate.
    const double robust_sigma = 1.4826 * mad;

    const double threshold =
        center + mad_scale * robust_sigma +
        std::max(1e-12, center * 1e-9);

    // Second pass: reject statistical outliers.
    std::vector<Correspondence> filtered;
    filtered.reserve(candidates.size());

    for (const auto& c : candidates) {
        if (c.squared_distance <= threshold) {
            filtered.push_back(c);
        }
    }

    return filtered;
}

}