#include "../include/lio/preprocessing/range_filter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
    lio::LidarConfig config;
    config.min_range = 1.0;
    config.max_range = 10.0;

    lio::RangeFilter filter(config);

    lio::PointCloud input;

    // Inside range
    input.push_back({Eigen::Vector3d(3, 0, 0), 0.5f});

    // Below minimum range
    input.push_back({Eigen::Vector3d(0.5, 0, 0), 0.5f});

    // Above maximum range
    input.push_back({Eigen::Vector3d(11, 0, 0), 0.5f});

    // Exactly at minimum range
    input.push_back({Eigen::Vector3d(1, 0, 0), 0.5f});

    // Exactly at maximum range
    input.push_back({Eigen::Vector3d(10, 0, 0), 0.5f});

    // Invalid point
    input.push_back({
        Eigen::Vector3d(
            std::numeric_limits<double>::quiet_NaN(), 0, 0
        ),
        0.5f
    });

    const lio::PointCloud output = filter.filter(input);

    assert(output.size() == 3);

    assert(std::abs(output[0].xyz.x() - 3.0) < 1e-9);
    assert(std::abs(output[1].xyz.x() - 1.0) < 1e-9);
    assert(std::abs(output[2].xyz.x() - 10.0) < 1e-9);

    std::cout << "All range filter tests passed!\n";

    return 0;
}