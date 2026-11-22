#include "../include/lio/registration/registration_evaluator.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    lio::PointCloud source;
    lio::PointCloud target;

    source.push_back({Eigen::Vector3d(0, 0, 0), 1.0f});
    source.push_back({Eigen::Vector3d(1, 0, 0), 1.0f});
    source.push_back({Eigen::Vector3d(0, 1, 0), 1.0f});
    source.push_back({Eigen::Vector3d(1, 1, 0), 1.0f});

    target.push_back({Eigen::Vector3d(0.1, 0, 0), 1.0f});
    target.push_back({Eigen::Vector3d(1.1, 0, 0), 1.0f});
    target.push_back({Eigen::Vector3d(0.1, 1, 0), 1.0f});
    target.push_back({Eigen::Vector3d(1.1, 1, 0), 1.0f});

    std::vector<lio::Correspondence> correspondences = {
        {0, 0, 0.01},
        {1, 1, 0.01},
        {2, 2, 0.01}
    };

    const auto metrics = lio::RegistrationEvaluator::evaluate(
        source,
        target,
        correspondences,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );

    assert(metrics.valid);
    assert(metrics.inlier_count == 3);
    assert(metrics.source_count == 4);
    assert(std::abs(metrics.fitness_score - 0.75) < 1e-9);
    assert(std::abs(metrics.rmse - 0.1) < 1e-9);
    assert(std::abs(metrics.mean_error - 0.1) < 1e-9);

    // Empty correspondences should produce invalid metrics.
    const std::vector<lio::Correspondence> empty;

    const auto empty_metrics = lio::RegistrationEvaluator::evaluate(
        source,
        target,
        empty,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );

    assert(!empty_metrics.valid);
    assert(empty_metrics.inlier_count == 0);

    std::cout << "RMSE: " << metrics.rmse << '\n';
    std::cout << "Fitness: " << metrics.fitness_score << '\n';
    std::cout << "All registration evaluator tests passed!\n";

    return 0;
}