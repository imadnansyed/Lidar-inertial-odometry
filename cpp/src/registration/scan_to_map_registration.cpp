#include "../include/lio/registration/scan_to_map_registration.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace lio {

ScanToMapRegistration::ScanToMapRegistration(
    const LocalMap& map,
    const ScanToMapConfig& config
)
    : map_(map),
      config_(config)
{
    validateConfig();
}

void ScanToMapRegistration::validateConfig() const
{
    if (config_.max_iterations == 0) {
        throw std::invalid_argument(
            "Maximum iterations must be greater than zero"
        );
    }

    if (config_.min_correspondences < 3) {
        throw std::invalid_argument(
            "At least three correspondences are required"
        );
    }

    if (config_.max_correspondence_distance <= 0.0 ||
        config_.translation_epsilon <= 0.0 ||
        config_.rotation_epsilon <= 0.0) {
        throw std::invalid_argument(
            "Registration thresholds must be positive"
        );
    }
}

ScanToMapResult ScanToMapRegistration::align(
    const PointCloud& scan,
    const Eigen::Matrix3d& initial_R,
    const Eigen::Vector3d& initial_t
) const
{
    ScanToMapResult result;

    result.R = initial_R;
    result.t = initial_t;

    if (!initial_R.allFinite() || !initial_t.allFinite()) {
        throw std::invalid_argument(
            "Invalid initial pose"
        );
    }

    if (scan.empty() || map_.empty()) {
        return result;
    }

    for (std::size_t iteration = 0;
         iteration < config_.max_iterations;
         ++iteration) {

        std::vector<Eigen::Vector3d> source_points;
        std::vector<Eigen::Vector3d> target_points;

        source_points.reserve(scan.size());
        target_points.reserve(scan.size());

        // 1. Find correspondences using the current pose.
        for (const auto& point : scan) {
            if (!point.xyz.allFinite()) {
                continue;
            }

            const Eigen::Vector3d point_map =
                result.R * point.xyz + result.t;

            const auto neighbors = map_.nearestKSearch(
                point_map,
                1,
                config_.max_correspondence_distance
            );

            if (neighbors.empty()) {
                continue;
            }

            source_points.push_back(point.xyz);
            target_points.push_back(neighbors[0].point);
            
        }

        result.iterations = iteration + 1;
        result.num_correspondences = source_points.size();

        if (source_points.size() < config_.min_correspondences) {
            break;
        }

        // 2. Calculate source and target centroids.
        Eigen::Vector3d source_mean =
            Eigen::Vector3d::Zero();

        Eigen::Vector3d target_mean =
            Eigen::Vector3d::Zero();

        for (std::size_t i = 0; i < source_points.size(); ++i) {
            source_mean += source_points[i];
            target_mean += target_points[i];
        }

        const double count =
            static_cast<double>(source_points.size());

        source_mean /= count;
        target_mean /= count;

        // 3. Build covariance matrix.
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

        for (std::size_t i = 0; i < source_points.size(); ++i) {
            const Eigen::Vector3d p =
                source_points[i] - source_mean;

            const Eigen::Vector3d q =
                target_points[i] - target_mean;

            H += p * q.transpose();
        }

        // 4. Solve rigid transformation using SVD.
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            H,
            Eigen::ComputeFullU | Eigen::ComputeFullV
        );

        // Reject degenerate correspondences (e.g. points on a line).
        if (svd.singularValues()(1) < 1e-12) {
            break;
        }

        const Eigen::Matrix3d U = svd.matrixU();
        const Eigen::Matrix3d V = svd.matrixV();

        Eigen::Matrix3d correction =
            Eigen::Matrix3d::Identity();

        if ((V * U.transpose()).determinant() < 0.0) {
            correction(2, 2) = -1.0;
        }

        const Eigen::Matrix3d new_R =
            V * correction * U.transpose();

        const Eigen::Vector3d new_t =
            target_mean - new_R * source_mean;

        // 5. Measure pose change.
        const Eigen::Matrix3d delta_R =
            new_R * result.R.transpose();

        const double cos_angle = std::clamp(
            (delta_R.trace() - 1.0) * 0.5,
            -1.0,
            1.0
        );

        const double rotation_change =
            std::acos(cos_angle);

        const double translation_change =
            (new_t - result.t).norm();

        // 6. Calculate mean squared alignment error.
        double squared_error = 0.0;

        for (std::size_t i = 0; i < source_points.size(); ++i) {
            const Eigen::Vector3d estimated =
                new_R * source_points[i] + new_t;

            squared_error +=
                (estimated - target_points[i]).squaredNorm();
        }

        result.fitness = squared_error / count;

        result.R = new_R;
        result.t = new_t;
        result.success = true;

        // 7. Check convergence.
        if (translation_change < config_.translation_epsilon &&
            rotation_change < config_.rotation_epsilon) {

            result.converged = true;
            break;
        }
    }

    return result;
}

}