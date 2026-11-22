#include "../include/lio/preprocessing/invalid_point_filter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
    lio::PointCloud input;

    // Valid point
    input.push_back({
        Eigen::Vector3d(1, 2, 3), 0.5f
    });

    // Invalid X
    input.push_back({
        Eigen::Vector3d(
            std::numeric_limits<double>::quiet_NaN(), 2, 3
        ), 0.5f
    });

    // Invalid Y
    input.push_back({
        Eigen::Vector3d(
            1, std::numeric_limits<double>::infinity(), 3
        ), 0.5f
    });

    // Invalid Z
    input.push_back({
        Eigen::Vector3d(
            1, 2, -std::numeric_limits<double>::infinity()
        ), 0.5f
    });

    // Invalid intensity
    input.push_back({
        Eigen::Vector3d(4, 5, 6),
        std::numeric_limits<float>::quiet_NaN()
    });

    // Another valid point
    input.push_back({
        Eigen::Vector3d(7, 8, 9), 1.0f
    });

    lio::InvalidPointFilter filter;

    const lio::PointCloud output = filter.filter(input);

    assert(output.size() == 2);

    assert(output[0].xyz.x() == 1.0);
    assert(output[1].xyz.x() == 7.0);

    std::cout << "All invalid point filter tests passed!\n";

    return 0;
}