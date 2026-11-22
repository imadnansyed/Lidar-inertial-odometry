#include "../include/lio/registration/point_to_point_icp.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    lio::PointCloud source;
    lio::PointCloud target;

    // Create an asymmetric source cloud.
    source.push_back({Eigen::Vector3d(0, 0, 0), 1.0f});
    source.push_back({Eigen::Vector3d(2, 0, 0), 1.0f});
    source.push_back({Eigen::Vector3d(0, 3, 0), 1.0f});
    source.push_back({Eigen::Vector3d(0, 0, 4), 1.0f});
    source.push_back({Eigen::Vector3d(2, 2, 1), 1.0f});
    source.push_back({Eigen::Vector3d(-1, 2, 3), 1.0f});

    // Ground-truth transformation.
    const Eigen::Matrix3d R_true =
        Eigen::AngleAxisd(
            0.05,
            Eigen::Vector3d::UnitZ()
        ).toRotationMatrix();

    const Eigen::Vector3d t_true(0.1, 0.05, 0.02);

    // Generate target cloud.
    for (const auto& point : source) {
        const Eigen::Vector3d transformed =
            R_true * point.xyz + t_true;

        target.push_back({transformed, point.intensity});
    }

    lio::PointToPointICP icp;

    const lio::ICPResult result = icp.align(
        source,
        target,
        0.8,
        50,
        1e-8
    );

    // Check recovered transformation.
    assert(result.converged);

    assert(
        (result.R - R_true).norm() < 1e-5
    );

    assert(
        (result.t - t_true).norm() < 1e-5
    );

    assert(result.rmse < 1e-5);
    assert(result.num_correspondences == source.size());

    std::cout << "ICP iterations: "
              << result.iterations << '\n';

    std::cout << "ICP RMSE: "
              << result.rmse << '\n';

    std::cout << "All point-to-point ICP tests passed!\n";

    return 0;
}