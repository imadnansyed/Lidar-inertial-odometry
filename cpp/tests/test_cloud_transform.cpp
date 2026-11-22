#include "../include/lio/preprocessing/cloud_transform.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    lio::PointCloud input;

    input.push_back({
        Eigen::Vector3d(1, 0, 0), 0.5f
    });

    input.push_back({
        Eigen::Vector3d(0, 1, 0), 0.8f
    });

    // 90-degree rotation around Z
    Eigen::Matrix3d R =
        Eigen::AngleAxisd(
            M_PI / 2.0,
            Eigen::Vector3d::UnitZ()
        ).toRotationMatrix();

    // Translation
    Eigen::Vector3d t(1, 2, 3);

    lio::SE3 T(R, t);

    lio::CloudTransform transformer;

    const lio::PointCloud output =
        transformer.transform(input, T);

    assert(output.size() == 2);

    // First point: (1,0,0) -> (1,3,3)
    assert(std::abs(output[0].xyz.x() - 1.0) < 1e-9);
    assert(std::abs(output[0].xyz.y() - 3.0) < 1e-9);
    assert(std::abs(output[0].xyz.z() - 3.0) < 1e-9);

    // Second point: (0,1,0) -> (0,2,3)
    assert(std::abs(output[1].xyz.x() - 0.0) < 1e-9);
    assert(std::abs(output[1].xyz.y() - 2.0) < 1e-9);
    assert(std::abs(output[1].xyz.z() - 3.0) < 1e-9);

    // Intensity must remain unchanged
    assert(output[0].intensity == 0.5f);
    assert(output[1].intensity == 0.8f);

    std::cout << "All cloud transformation tests passed!\n";

    return 0;
}