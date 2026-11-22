#include "../include/lio/io/kitti_reader.hpp"

#include "../include/lio/registration/point_to_point_icp.hpp"
#include "../include/lio/registration/point_to_plane_icp.hpp"
#include "../include/lio/registration/correspondence.hpp"
#include "../include/lio/registration/registration_evaluator.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

lio::PointCloud downsample(
    const lio::PointCloud& cloud,
    std::size_t stride
)
{
    lio::PointCloud output;
    output.reserve(cloud.size() / stride + 1);

    for (std::size_t i = 0; i < cloud.size(); i += stride) {
        output.push_back(cloud[i]);
    }

    return output;
}

void printMetrics(
    const std::string& name,
    const lio::RegistrationMetrics& metrics,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    std::size_t iterations,
    bool converged
)
{
    std::cout << "\n--- " << name << " ---\n";
    std::cout << "Converged: " << std::boolalpha << converged << '\n';
    std::cout << "Iterations: " << iterations << '\n';
    std::cout << "Inliers: " << metrics.inlier_count << '\n';
    std::cout << "Fitness: " << metrics.fitness_score << '\n';
    std::cout << "RMSE: " << metrics.rmse << " m\n";
    std::cout << "Translation: " << t.transpose() << '\n';
    std::cout << "Rotation:\n" << R << '\n';
}

}

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr
            << "Usage: test_kitti_registration "
            << "<source_scan.bin> <target_scan.bin>\n";
        return 1;
    }

    try {
        // 1. Read consecutive KITTI scans.
        lio::KittiReader reader;

        const auto source_scan =
            reader.readVelodyneScan(argv[1]);

        const auto target_scan =
            reader.readVelodyneScan(argv[2]);

        std::cout << "Source points: "
                  << source_scan.points.size() << '\n';

        std::cout << "Target points: "
                  << target_scan.points.size() << '\n';

        // 2. Downsample both scans.
        const lio::PointCloud source =
            downsample(source_scan.points, 20);

        const lio::PointCloud target =
            downsample(target_scan.points, 20);

        std::cout << "Source sampled: " << source.size() << '\n';
        std::cout << "Target sampled: " << target.size() << '\n';

        if (source.size() < 10 || target.size() < 10) {
            throw std::runtime_error("Insufficient points in scans");
        }

        constexpr double max_distance = 2.0;

        // 3. Point-to-point ICP.
        lio::PointToPointICP point_to_point;

        const auto p2p_result = point_to_point.align(
            source,
            target,
            max_distance,
            50,
            1e-5
        );

        lio::CorrespondenceSearch p2p_search(target);

        const auto p2p_correspondences = p2p_search.find(
            source,
            p2p_result.R,
            p2p_result.t,
            max_distance
        );

        const auto p2p_metrics =
            lio::RegistrationEvaluator::evaluate(
                source,
                target,
                p2p_correspondences,
                p2p_result.R,
                p2p_result.t
            );

        printMetrics(
            "Point-to-Point ICP",
            p2p_metrics,
            p2p_result.R,
            p2p_result.t,
            p2p_result.iterations,
            p2p_result.converged
        );

        // 4. Point-to-plane ICP.
        lio::PointToPlaneICP point_to_plane;

        const auto p2plane_result = point_to_plane.align(
            source,
            target,
            max_distance,
            30,
            1e-5,
            10
        );

        lio::CorrespondenceSearch p2plane_search(target);

        const auto p2plane_correspondences = p2plane_search.find(
            source,
            p2plane_result.R,
            p2plane_result.t,
            max_distance
        );

        const auto p2plane_metrics =
            lio::RegistrationEvaluator::evaluate(
                source,
                target,
                p2plane_correspondences,
                p2plane_result.R,
                p2plane_result.t
            );

        printMetrics(
            "Point-to-Plane ICP",
            p2plane_metrics,
            p2plane_result.R,
            p2plane_result.t,
            p2plane_result.iterations,
            p2plane_result.converged
        );

        // 5. Basic validity checks.
        if (!p2p_result.R.allFinite() ||
            !p2p_result.t.allFinite() ||
            !p2p_metrics.valid) {
            throw std::runtime_error(
                "Point-to-point registration produced invalid results"
            );
        }

        if (!p2plane_result.R.allFinite() ||
            !p2plane_result.t.allFinite() ||
            !p2plane_metrics.valid) {
            throw std::runtime_error(
                "Point-to-plane registration produced invalid results"
            );
        }

        std::cout << "\nKITTI registration test completed!\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Registration test failed: "
                  << e.what() << '\n';
        return 1;
    }

    return 0;
}