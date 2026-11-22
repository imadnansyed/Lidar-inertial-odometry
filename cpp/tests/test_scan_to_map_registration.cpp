#include "../include/lio/registration/scan_to_map_registration.hpp"

#include <Eigen/Geometry>

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    // 1. Generate a synthetic scan.
    lio::PointCloud source_scan;
    lio::PointCloud target_map;

    const Eigen::Matrix3d true_R =
        Eigen::AngleAxisd(
            0.02,
            Eigen::Vector3d::UnitZ()
        ).toRotationMatrix();

    const Eigen::Vector3d true_t(0.08, -0.06, 0.04);

    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            for (int k = 0; k < 3; ++k) {

                const Eigen::Vector3d p(
                    i * 1.5 + 0.11 * j * k,
                    j * 1.3 + 0.07 * i * k,
                    k * 1.2 + 0.05 * i * j
                );

                const Eigen::Vector3d q =
                    true_R * p + true_t;

                source_scan.push_back({p, 1.0f});
                target_map.push_back({q, 1.0f});
            }
        }
    }

    // 2. Initialize local map with target points.
    lio::LocalMapConfig map_config;
    map_config.max_radius = 100.0;
    map_config.max_points = 1000;

    lio::LocalMap map(map_config);

    map.initialize(
        target_map,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );

    // 3. Configure scan-to-map registration.
    lio::ScanToMapConfig config;
    config.max_iterations = 30;
    config.min_correspondences = 10;
    config.max_correspondence_distance = 0.6;
    config.translation_epsilon = 1e-8;
    config.rotation_epsilon = 1e-8;

    lio::ScanToMapRegistration registration(map, config);

    // 4. Start with an approximate pose.
    const auto result = registration.align(
        source_scan,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );

    // 5. Validate estimated pose.
    assert(result.success);
    assert(result.converged);

    assert(
        (result.R - true_R).norm() < 1e-5
    );

    assert(
        (result.t - true_t).norm() < 1e-5
    );

    std::cout << "Correspondences: "
              << result.num_correspondences << '\n';

    std::cout << "Iterations: "
              << result.iterations << '\n';

    std::cout << "Fitness: "
              << result.fitness << '\n';

    std::cout << "Translation error: "
              << (result.t - true_t).norm() << '\n';

    std::cout << "Rotation matrix error: "
              << (result.R - true_R).norm() << '\n';

    std::cout << "Scan-to-map registration test passed!\n";

    return 0;
}