#include <iostream>
#include <cassert>
#include <cmath>

#include "lio/common/se3.hpp"

int main()
{
    using namespace lio;

    constexpr double PI = 3.14159265358979323846;

    // 90-degree rotation around Z-axis
    Eigen::Matrix3d R =
        Eigen::AngleAxisd(
            PI / 2.0,
            Eigen::Vector3d::UnitZ()
        ).toRotationMatrix();

    // Translation
    Eigen::Vector3d t(1.0, 2.0, 3.0);

    SE3 T_AB(R, t);

    // Test 1: Transform a point
    Eigen::Vector3d p_B(1.0, 0.0, 0.0);

    Eigen::Vector3d p_A = T_AB * p_B;

    Eigen::Vector3d expected(1.0, 3.0, 3.0);

    assert(p_A.isApprox(expected, 1e-9));

    // Test 2: Inverse transformation
    SE3 T_BA = T_AB.inverse();

    Eigen::Vector3d recovered = T_BA * p_A;

    assert(recovered.isApprox(p_B, 1e-9));

    // Test 3: Composition with inverse = identity
    SE3 identity = T_AB * T_BA;

    assert(identity.rotation().isApprox(
        Eigen::Matrix3d::Identity(), 1e-9
    ));

    assert(identity.translation().isApprox(
        Eigen::Vector3d::Zero(), 1e-9
    ));

    // Test 4: Homogeneous matrix
    Eigen::Matrix4d T_matrix = T_AB.matrix();

    assert((T_matrix.block<3, 3>(0, 0).isApprox(R)));

    assert((T_matrix.block<3, 1>(0, 3).isApprox(t)));

    std::cout << "All SE3 tests passed!\n";

    return 0;
}