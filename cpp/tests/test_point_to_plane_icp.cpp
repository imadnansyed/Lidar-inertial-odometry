#include "../include/lio/registration/point_to_plane_icp.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    lio::PointCloud target;
    lio::PointCloud source;

    // Create a 2D grid on the target plane (z = 0).
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 10; ++y) {
            Eigen::Vector3d p(
                x * 0.2,
                y * 0.2,
                0.0
            );

            target.push_back({p, 1.0f});

            // Source plane is shifted upward by 5 cm.
            p.z() += 0.05;

            source.push_back({p, 1.0f});
        }
    }

    lio::PointToPlaneICP icp;

    const auto result = icp.align(
        source,
        target,
        0.2,
        30,
        1e-8,
        10
    );

    assert(result.converged);
    assert(std::abs(result.t.z() + 0.05) < 1e-5);
    assert(std::abs(result.rmse) < 1e-5);
    assert(result.num_correspondences >= 6);

    std::cout << "Iterations: " << result.iterations << '\n';
    std::cout << "RMSE: " << result.rmse << '\n';
    std::cout << "Estimated translation: "
              << result.t.transpose() << '\n';

    std::cout << "All point-to-plane ICP tests passed!\n";

    return 0;
}