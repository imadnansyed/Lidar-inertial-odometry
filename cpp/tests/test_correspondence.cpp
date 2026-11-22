#include "../include/lio/registration/correspondence.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    // Target cloud.
    lio::PointCloud target;

    target.push_back({Eigen::Vector3d(0, 0, 0), 1.0f});
    target.push_back({Eigen::Vector3d(2, 0, 0), 2.0f});
    target.push_back({Eigen::Vector3d(0, 2, 0), 3.0f});

    // Source cloud.
    lio::PointCloud source;

    source.push_back({Eigen::Vector3d(0.1, 0, 0), 1.0f});
    source.push_back({Eigen::Vector3d(2.1, 0, 0), 2.0f});
    source.push_back({Eigen::Vector3d(10, 10, 10), 3.0f});

    lio::CorrespondenceSearch search(target);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    auto correspondences = search.find(
        source,
        R,
        t,
        0.5
    );

    assert(correspondences.size() == 2);

    assert(correspondences[0].source_index == 0);
    assert(correspondences[0].target_index == 0);
    assert(std::abs(
        correspondences[0].squared_distance - 0.01
    ) < 1e-9);

    assert(correspondences[1].source_index == 1);
    assert(correspondences[1].target_index == 1);

    // Test with translation.
    t = Eigen::Vector3d(-0.1, 0, 0);

    correspondences = search.find(source, R, t, 0.5);

    assert(correspondences.size() == 2);
    assert(correspondences[0].target_index == 0);

    std::cout << "All correspondence tests passed!\n";

    return 0;
}