#include "../include/lio/odometry/relative_pose.hpp"

#include <stdexcept>

namespace lio {

Eigen::Matrix4d RelativePoseEstimate::matrix() const
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;

    return T;
}

RelativePoseEstimate RelativePoseEstimate::inverse() const
{
    RelativePoseEstimate result;

    result.R = R.transpose();
    result.t = -result.R * t;

    result.rmse = rmse;
    result.iterations = iterations;
    result.num_correspondences = num_correspondences;
    result.converged = converged;

    return result;
}

RelativePoseEstimate RelativePoseEstimator::estimate(
    const PointCloud& source,
    const PointCloud& target,
    double max_correspondence_distance,
    std::size_t max_iterations,
    double tolerance
) const
{
    PointToPointICP icp;

    const ICPResult icp_result = icp.align(
        source,
        target,
        max_correspondence_distance,
        max_iterations,
        tolerance
    );

    RelativePoseEstimate result;

    result.R = icp_result.R;
    result.t = icp_result.t;
    result.rmse = icp_result.rmse;
    result.iterations = icp_result.iterations;
    result.num_correspondences =
        icp_result.num_correspondences;
    result.converged = icp_result.converged;

    if (!result.R.allFinite() || !result.t.allFinite()) {
        throw std::runtime_error(
            "Relative pose estimation produced invalid transformation"
        );
    }

    return result;
}

}