#include "../include/lio/registration/registration_evaluator.hpp"

#include <cmath>
#include <stdexcept>

namespace lio {

RegistrationMetrics RegistrationEvaluator::evaluate(
    const PointCloud& source,
    const PointCloud& target,
    const std::vector<Correspondence>& correspondences,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t
)
{
    if (!R.allFinite() || !t.allFinite()) {
        throw std::invalid_argument("Invalid registration transformation");
    }

    RegistrationMetrics metrics;
    metrics.source_count = source.size();

    if (source.empty() || target.empty() || correspondences.empty()) {
        return metrics;
    }

    double squared_error_sum = 0.0;
    double error_sum = 0.0;

    for (const auto& c : correspondences) {
        if (c.source_index >= source.size() ||
            c.target_index >= target.size()) {
            throw std::out_of_range("Correspondence index out of range");
        }

        const Eigen::Vector3d transformed =
            R * source[c.source_index].xyz + t;

        const Eigen::Vector3d error =
            transformed - target[c.target_index].xyz;

        const double distance = error.norm();

        if (!std::isfinite(distance)) {
            throw std::runtime_error("Invalid registration error");
        }

        squared_error_sum += distance * distance;
        error_sum += distance;
    }

    metrics.inlier_count = correspondences.size();

    metrics.rmse = std::sqrt(
        squared_error_sum /
        static_cast<double>(metrics.inlier_count)
    );

    metrics.mean_error =
        error_sum / static_cast<double>(metrics.inlier_count);

    metrics.fitness_score =
        static_cast<double>(metrics.inlier_count) /
        static_cast<double>(metrics.source_count);

    metrics.valid = true;

    return metrics;
}

}