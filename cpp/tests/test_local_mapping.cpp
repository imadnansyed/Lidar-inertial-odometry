#include "../include/lio/mapping/local_mapping.hpp"

#include <Eigen/Geometry>

#include <cassert>
#include <iostream>

int main()
{
    lio::PointCloud first_scan;
    lio::PointCloud second_scan;

    const Eigen::Matrix3d true_R =
        Eigen::AngleAxisd(
            0.02,
            Eigen::Vector3d::UnitZ()
        ).toRotationMatrix();

    const Eigen::Vector3d true_t(0.08, -0.06, 0.04);

    // Generate a synthetic environment.
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            for (int k = 0; k < 3; ++k) {

                const Eigen::Vector3d world_point(
                    i * 1.5 + 0.11 * j * k,
                    j * 1.3 + 0.07 * i * k,
                    k * 1.2 + 0.05 * i * j
                );

                // First scan is already in the map frame.
                first_scan.push_back({
                    true_R * world_point + true_t,
                    1.0f
                });

                // Second scan observes the same environment
                // from a different pose.
                second_scan.push_back({
                    world_point,
                    1.0f
                });
            }
        }
    }

    // Configure the mapping pipeline.
    lio::LocalMappingConfig config;

    config.map.max_radius = 100.0;
    config.map.max_points = 1000;

    config.registration.max_iterations = 30;
    config.registration.min_correspondences = 10;
    config.registration.max_correspondence_distance = 0.6;
    config.registration.translation_epsilon = 1e-8;
    config.registration.rotation_epsilon = 1e-8;

    lio::LocalMapping mapping(config);

    // 1. Process first scan and initialize map.
    const auto first_result = mapping.processScan(
        first_scan,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );

    assert(first_result.initialized_map);
    assert(first_result.scan_inserted);
    assert(!first_result.registration_attempted);

    assert(mapping.map().size() == first_scan.size());

    std::cout << "First scan initialization passed\n";

    // 2. Process second scan against the map.
    const auto second_result = mapping.processScan(
        second_scan,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );

    assert(second_result.registration_attempted);
    assert(second_result.registration.success);
    assert(second_result.registration.converged);
    assert(second_result.scan_inserted);

    // 3. Check estimated pose.
    assert(
        (second_result.R - true_R).norm() < 1e-5
    );

    assert(
        (second_result.t - true_t).norm() < 1e-5
    );

    // 4. Check map update.
    assert(
        mapping.map().size() ==
        first_scan.size() + second_scan.size()
    );

    std::cout << "Scan-to-map registration passed\n";

    std::cout << "Map insertion passed\n";

    std::cout << "Final map points: "
              << mapping.map().size() << '\n';

    std::cout << "Translation error: "
              << (second_result.t - true_t).norm() << '\n';

    std::cout << "All local mapping tests passed!\n";

    return 0;
}