#include "../include/lio/registration/nearest_neighbor.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main()
{
    lio::PointCloud cloud;

    cloud.push_back({Eigen::Vector3d(0, 0, 0), 1.0f});
    cloud.push_back({Eigen::Vector3d(2, 0, 0), 2.0f});
    cloud.push_back({Eigen::Vector3d(0, 3, 0), 3.0f});

    lio::NearestNeighbor search(cloud);

    // Test nearest neighbor
    Eigen::Vector3d query(1.8, 0.1, 0.0);

    auto result = search.search(query);

    assert(result.has_value());
    assert(result->index == 1);

    assert(std::abs(
        result->squared_distance - 0.05
    ) < 1e-9);

    // Test maximum search distance
    auto rejected = search.search(
        Eigen::Vector3d(5, 5, 5),
        1.0
    );

    assert(!rejected.has_value());

    // Test empty cloud
    bool exception_thrown = false;

    try {
        lio::PointCloud empty;
        lio::NearestNeighbor invalid_search(empty);
    }
    catch (const std::invalid_argument&) {
        exception_thrown = true;
    }

    assert(exception_thrown);

    std::cout << "All nearest neighbor tests passed!\n";

    return 0;
}