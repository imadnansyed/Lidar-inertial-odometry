#include "../include/lio/odometry/relative_pose.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    lio::PointCloud source;
    lio::PointCloud target;

    // Create asymmetric source cloud.
    source.push_back({Eigen::Vector3d(0, 0, 0), 1.0f});
    source.push_back({Eigen::Vector3d(2, 0, 0), 1.0f});
    source.push_back({Eigen::Vector3d(0, 3, 0), 1.0f});
    source.push_back({Eigen::Vector3d(0, 0, 4), 1.0f});
    source.push_back({Eigen::Vector3d(2, 2, 1), 1.0f});
    source.push_back({Eigen::Vector3d(-1, 2, 3), 1.0f});

    // Known relative transformation.
    const Eigen::Matrix3d R_true =
        Eigen::AngleAxisd(
            0.05,
            Eigen::Vector3d::UnitZ()
        ).toRotationMatrix();

    const Eigen::Vector3d t_true(0.1, 0.05, 0.02);

    for (const auto& point : source) {
        target.push_back({
            R_true * point.xyz + t_true,
            point.intensity
        });
    }

    // Estimate relative pose.
    lio::RelativePoseEstimator estimator;

    const auto result = estimator.estimate(
        source,
        target,
        0.8,
        50,
        1e-8
    );

    assert(result.converged);

    assert((result.R - R_true).norm() < 1e-5);
    assert((result.t - t_true).norm() < 1e-5);

    // Test homogeneous transformation.
    const Eigen::Matrix4d T = result.matrix();

    Eigen::Vector4d p(1, 2, 3, 1);

    const Eigen::Vector4d transformed = T * p;

    const Eigen::Vector3d expected =
        R_true * p.head<3>() + t_true;

    assert(
        (transformed.head<3>() - expected).norm() < 1e-5
    );

    // Test inverse transformation.
    const auto inverse = result.inverse();

    const Eigen::Matrix4d identity =
        inverse.matrix() * result.matrix();

    assert(
        (identity - Eigen::Matrix4d::Identity()).norm() < 1e-8
    );

    std::cout << "Estimated relative transformation:\n";
    std::cout << result.matrix() << '\n';

    std::cout << "\nAll relative pose tests passed!\n";

    return 0;
}