#include "../include/lio/registration/point_to_point_icp.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <stdexcept>



namespace lio {

namespace {

struct RigidTransform {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
};

// Estimate the best rigid transformation using the Kabsch algorithm.
RigidTransform estimateTransform(
    const PointCloud& source,
    const PointCloud& target,
    const std::vector<Correspondence>& correspondences
)
{
    if (correspondences.size() < 3) {
        throw std::runtime_error(
            "At least 3 correspondences are required"
        );
    }

    Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_centroid = Eigen::Vector3d::Zero();

    for (const auto& c : correspondences) {
        source_centroid += source[c.source_index].xyz;
        target_centroid += target[c.target_index].xyz;
    }

    const double count =
        static_cast<double>(correspondences.size());

    source_centroid /= count;
    target_centroid /= count;

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    for (const auto& c : correspondences) {
        const Eigen::Vector3d p =
            source[c.source_index].xyz - source_centroid;

        const Eigen::Vector3d q =
            target[c.target_index].xyz - target_centroid;

        H += p * q.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        H,
        Eigen::ComputeFullU | Eigen::ComputeFullV
    );

    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();

    if ((V * U.transpose()).determinant() < 0.0) {
        correction(2, 2) = -1.0;
    }

    RigidTransform transform;

    transform.R = V * correction * U.transpose();

    transform.t =
        target_centroid - transform.R * source_centroid;

    return transform;
}

double calculateRMSE(
    const PointCloud& source,
    const PointCloud& target,
    const std::vector<Correspondence>& correspondences,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t
)
{
    double squared_error = 0.0;

    for (const auto& c : correspondences) {
        const Eigen::Vector3d transformed =
            R * source[c.source_index].xyz + t;

        const Eigen::Vector3d error =
            transformed - target[c.target_index].xyz;

        squared_error += error.squaredNorm();
    }

    return std::sqrt(
        squared_error /
        static_cast<double>(correspondences.size())
    );
}

} // anonymous namespace


ICPResult PointToPointICP::align(
    const PointCloud& source,
    const PointCloud& target,
    double max_correspondence_distance,
    std::size_t max_iterations,
    double tolerance
) const
{
    if (source.size() < 3 || target.size() < 3) {
        throw std::invalid_argument(
            "Source and target clouds must contain at least 3 points"
        );
    }

    if (!std::isfinite(max_correspondence_distance) ||
        max_correspondence_distance <= 0.0) {
        throw std::invalid_argument(
            "Maximum correspondence distance must be positive and finite"
        );
    }

    if (max_iterations == 0 ||
        !std::isfinite(tolerance) ||
        tolerance <= 0.0) {
        throw std::invalid_argument("Invalid ICP convergence settings");
    }

    // Build target KD-tree once.
    CorrespondenceSearch correspondence_search(target);

    ICPResult result;

    for (std::size_t iteration = 0;
         iteration < max_iterations;
         ++iteration) {

        // 1. Find correspondences using the current pose.
        const auto correspondences = correspondence_search.find(
            source,
            result.R,
            result.t,
            max_correspondence_distance
        );

        if (correspondences.size() < 3) {
            throw std::runtime_error(
                "ICP found fewer than 3 valid correspondences"
            );
        }

        // 2. Estimate transformation from original source points.
        const RigidTransform updated = estimateTransform(
            source,
            target,
            correspondences
        );

        // 3. Measure the change in pose.
        const Eigen::Matrix3d delta_R =
            updated.R * result.R.transpose();

        const Eigen::Vector3d delta_t =
            updated.t - result.t;

        const double cos_angle = std::clamp(
                                (delta_R.trace() - 1.0) * 0.5,
                                -1.0,
                                1.0
                            );

        const Eigen::Vector3d skew(
            delta_R(2, 1) - delta_R(1, 2),
            delta_R(0, 2) - delta_R(2, 0),
            delta_R(1, 0) - delta_R(0, 1)
        );

        const double sin_angle = 0.5 * skew.norm();

        // using std::atan2 to compute the rotation change in a numerically stable way
        const double rotation_change = std::atan2(
                                sin_angle,
                                cos_angle
                            );

        const double translation_change = delta_t.norm();

        // Update current pose.
        result.R = updated.R;
        result.t = updated.t;

        result.rmse = calculateRMSE(
            source,
            target,
            correspondences,
            result.R,
            result.t
        );

        result.iterations = iteration + 1;
        result.num_correspondences = correspondences.size();

        // 4. Check convergence.
        if (translation_change < tolerance &&
            rotation_change < tolerance) {

            result.converged = true;
            break;
        }
    }

    return result;
}

}