#include <iostream>
#include <cassert>
#include <cmath>

#include "../include/lio/common/math.hpp"

int main()
{
    using namespace lio;
    using namespace lio::math;

    const double pi = std::acos(-1.0);

    // Test 1: Skew-symmetric matrix
    Eigen::Vector3d v(1.0, 2.0, 3.0);
    Eigen::Vector3d u(4.0, 5.0, 6.0);

    Eigen::Vector3d result = skew(v) * u;
    Eigen::Vector3d expected = v.cross(u);

    assert(result.isApprox(expected, 1e-9));

    // Test 2: Exponential map
    Eigen::Vector3d omega(0.0, 0.0, pi / 2.0);

    Eigen::Matrix3d R = Exp(omega);

    Eigen::Vector3d point(1.0, 0.0, 0.0);
    Eigen::Vector3d rotated = R * point;

    Eigen::Vector3d expected_rotated(0.0, 1.0, 0.0);

    assert(rotated.isApprox(expected_rotated, 1e-9));

    // Test 3: Logarithm map
    Eigen::Vector3d recovered_omega = Log(R);

    assert(recovered_omega.isApprox(omega, 1e-9));

    // Test 4: Small-angle exponential
    Eigen::Vector3d small_omega(1e-10, 2e-10, 3e-10);

    Eigen::Matrix3d R_small = Exp(small_omega);

    assert(R_small.allFinite());

    // Test 5: Rotation normalization
    Eigen::Matrix3d noisy_R = R;
    noisy_R(0, 0) += 0.001;

    Eigen::Matrix3d R_normalized = normalizeRotation(noisy_R);

    Eigen::Matrix3d orthogonality =
        R_normalized.transpose() * R_normalized;

    assert(orthogonality.isApprox(
        Eigen::Matrix3d::Identity(), 1e-9
    ));

    assert(std::abs(R_normalized.determinant() - 1.0) < 1e-9);

    std::cout << "All SO3 math tests passed!\n";

    return 0;
}