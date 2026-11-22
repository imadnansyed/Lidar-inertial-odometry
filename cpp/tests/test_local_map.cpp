
#include "../include/lio/mapping/local_map.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    lio::LocalMapConfig config;
    config.max_radius = 5.0;
    config.max_points = 100;

    lio::LocalMap map(config);

    // --------------------------------
    // Test 5.1: Initial map state
    // --------------------------------

    assert(map.empty());
    assert(!map.initialized());

    // --------------------------------
    // Test 5.2: Initialize from scan 1
    // --------------------------------

    lio::PointCloud scan1;

    scan1.push_back({
        Eigen::Vector3d(0, 0, 0), 1.0f
    });

    scan1.push_back({
        Eigen::Vector3d(1, 0, 0), 1.0f
    });

    scan1.push_back({
        Eigen::Vector3d(0, 1, 0), 1.0f
    });

    const Eigen::Vector3d pose1(0, 0, 0);

    map.initialize(
        scan1,
        Eigen::Matrix3d::Identity(),
        pose1
    );

    assert(map.initialized());
    assert(map.size() == 3);

    // --------------------------------
    // Test 5.3: Insert scan 2
    // --------------------------------

    lio::PointCloud scan2;

    scan2.push_back({
        Eigen::Vector3d(0, 0, 0), 2.0f
    });

    scan2.push_back({
        Eigen::Vector3d(1, 0, 0), 2.0f
    });

    const Eigen::Vector3d pose2(2, 0, 0);

    map.insertScan(
        scan2,
        Eigen::Matrix3d::Identity(),
        pose2
    );

    assert(map.size() == 5);

    // First inserted point from scan 2
    // should be located at (2, 0, 0).

    assert(
        (
            map.points()[3].xyz -
            Eigen::Vector3d(2, 0, 0)
        ).norm() < 1e-8
    );

    // --------------------------------
    // Test 5.4: Nearest-neighbor search
    // --------------------------------

    const auto neighbors = map.nearestKSearch(
        Eigen::Vector3d(2, 0, 0),
        2,
        1.1
    );

    assert(neighbors.size() == 2);

    assert(
        neighbors[0].squared_distance <=
        neighbors[1].squared_distance
    );

    assert(
        neighbors[0].squared_distance < 1e-8
    );

    // --------------------------------
    // Test: Local map pruning
    // --------------------------------

    map.prune(Eigen::Vector3d(100, 0, 0));

    assert(map.empty());
    assert(map.initialized());

    // --------------------------------
    // Test: Clear
    // --------------------------------

    map.clear();

    assert(map.empty());
    assert(!map.initialized());

    std::cout << "Local map size: " << map.size() << '\n';

    std::cout << "All local map tests passed!\n";

    return 0;
}
