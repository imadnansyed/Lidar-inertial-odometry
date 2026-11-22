#include "../include/lio/odometry/trajectory_evaluator.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <cmath>
#include <stdexcept>
#include <vector>
#include <algorithm>

namespace lio {

namespace {

SE3 estimateAlignment(
    const Trajectory& estimated,
    const Trajectory& ground_truth
)
{
    const std::size_t N = estimated.size();

    Eigen::Vector3d mean_est = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gt = Eigen::Vector3d::Zero();

    for (std::size_t i = 0; i < N; ++i) {
        mean_est += estimated.at(i).pose.translation();
        mean_gt += ground_truth.at(i).pose.translation();
    }

    mean_est /= static_cast<double>(N);
    mean_gt /= static_cast<double>(N);

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    for (std::size_t i = 0; i < N; ++i) {
        const Eigen::Vector3d p =
            estimated.at(i).pose.translation() - mean_est;

        const Eigen::Vector3d q =
            ground_truth.at(i).pose.translation() - mean_gt;

        H += p * q.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        H,
        Eigen::ComputeFullU | Eigen::ComputeFullV
    );

    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R = V * U.transpose();

    if (R.determinant() < 0.0) {
        Eigen::Matrix3d V_corrected = V;
        V_corrected.col(2) *= -1.0;

        R = V_corrected * U.transpose();
    }

    const Eigen::Vector3d t = mean_gt - R * mean_est;

    return SE3(R, t);
}

double rotationError(const Eigen::Matrix3d& R)
{
    const double cosine = std::clamp(
        (R.trace() - 1.0) * 0.5,
        -1.0,
        1.0
    );

    return std::acos(cosine);
}

}

TrajectoryMetrics TrajectoryEvaluator::evaluate(
    const Trajectory& estimated,
    const Trajectory& ground_truth,
    std::size_t rpe_delta
)
{
    if (estimated.size() != ground_truth.size()) {
        throw std::invalid_argument(
            "Estimated and ground-truth trajectories must have equal sizes"
        );
    }

    if (estimated.size() < 2) {
        throw std::invalid_argument(
            "At least two poses are required"
        );
    }

    if (rpe_delta == 0 || rpe_delta >= estimated.size()) {
        throw std::invalid_argument(
            "Invalid RPE delta"
        );
    }

    const std::size_t N = estimated.size();

    TrajectoryMetrics metrics;

    metrics.num_poses = N;

    // Estimate rigid alignment.
    metrics.alignment = estimateAlignment(
        estimated,
        ground_truth
    );

    std::vector<SE3> aligned_estimated;
    aligned_estimated.reserve(N);

    for (std::size_t i = 0; i < N; ++i) {
        aligned_estimated.push_back(
            metrics.alignment * estimated.at(i).pose
        );
    }

    // ATE.
    double ate_squared_sum = 0.0;

    for (std::size_t i = 0; i < N; ++i) {
        const Eigen::Vector3d error =
            aligned_estimated[i].translation() -
            ground_truth.at(i).pose.translation();

        ate_squared_sum += error.squaredNorm();
    }

    metrics.ate_rmse = std::sqrt(
        ate_squared_sum / static_cast<double>(N)
    );

    // RPE.
    double translation_squared_sum = 0.0;
    double rotation_squared_sum = 0.0;

    for (std::size_t i = 0; i + rpe_delta < N; ++i) {
        const std::size_t j = i + rpe_delta;

        const SE3 estimated_relative =
            aligned_estimated[i].inverse() *
            aligned_estimated[j];

        const SE3 ground_truth_relative =
            ground_truth.at(i).pose.inverse() *
            ground_truth.at(j).pose;

        const SE3 error =
            ground_truth_relative.inverse() *
            estimated_relative;

        translation_squared_sum +=
            error.translation().squaredNorm();

        const double angle = rotationError(error.rotation());

        rotation_squared_sum += angle * angle;

        ++metrics.num_rpe_pairs;
    }

    const double pair_count =
        static_cast<double>(metrics.num_rpe_pairs);

    metrics.rpe_translation_rmse =
        std::sqrt(translation_squared_sum / pair_count);

    metrics.rpe_rotation_rmse_rad =
        std::sqrt(rotation_squared_sum / pair_count);

    return metrics;
}

}