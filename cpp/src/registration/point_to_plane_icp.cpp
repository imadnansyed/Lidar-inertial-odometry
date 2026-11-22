#include "../include/lio/registration/point_to_plane_icp.hpp"

#include "../include/lio/registration/correspondence.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lio {

namespace {

std::vector<Eigen::Vector3d> estimateNormals(
    const PointCloud& cloud,
    const NearestNeighbor& search,
    std::size_t k
)
{
    std::vector<Eigen::Vector3d> normals(cloud.size());

    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto neighbors = search.searchK(cloud[i].xyz, k);

        if (neighbors.size() < 3) {
            throw std::runtime_error("Insufficient neighbors for normal estimation");
        }

        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

        for (const auto& neighbor : neighbors) {
            centroid += cloud[neighbor.index].xyz;
        }

        centroid /= static_cast<double>(neighbors.size());

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

        for (const auto& neighbor : neighbors) {
            const Eigen::Vector3d d =
                cloud[neighbor.index].xyz - centroid;

            covariance += d * d.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("Normal estimation failed");
        }

        Eigen::Vector3d normal = solver.eigenvectors().col(0);

        if (normal.norm() < 1e-12) {
            throw std::runtime_error("Invalid surface normal");
        }

        normals[i] = normal.normalized();
    }

    return normals;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d matrix;

    matrix <<  0.0, -v.z(),  v.y(),
               v.z(),  0.0, -v.x(),
              -v.y(),  v.x(),  0.0;

    return matrix;
}

} // anonymous namespace


PointToPlaneResult PointToPlaneICP::align(
    const PointCloud& source,
    const PointCloud& target,
    double max_correspondence_distance,
    std::size_t max_iterations,
    double tolerance,
    std::size_t normal_neighbors
) const
{
    if (source.size() < 3 || target.size() < 3) {
        throw std::invalid_argument("Point clouds need at least 3 points");
    }

    if (!std::isfinite(max_correspondence_distance) ||
        max_correspondence_distance <= 0.0 ||
        max_iterations == 0 ||
        !std::isfinite(tolerance) ||
        tolerance <= 0.0 ||
        normal_neighbors < 3) {
        throw std::invalid_argument("Invalid point-to-plane ICP settings");
    }

    NearestNeighbor target_search(target);

    const auto normals = estimateNormals(
        target,
        target_search,
        normal_neighbors
    );

    CorrespondenceSearch correspondence_search(target);

    PointToPlaneResult result;

    for (std::size_t iteration = 0;
         iteration < max_iterations;
         ++iteration) {

        const auto correspondences = correspondence_search.find(
            source,
            result.R,
            result.t,
            max_correspondence_distance
        );

        if (correspondences.size() < 6) {
            throw std::runtime_error(
                "Insufficient correspondences for point-to-plane ICP"
            );
        }

        Eigen::Matrix<double, 6, 6> H =
            Eigen::Matrix<double, 6, 6>::Zero();

        Eigen::Matrix<double, 6, 1> g =
            Eigen::Matrix<double, 6, 1>::Zero();

        double squared_error = 0.0;

        for (const auto& c : correspondences) {
            const Eigen::Vector3d p =
                result.R * source[c.source_index].xyz + result.t;

            const Eigen::Vector3d q =
                target[c.target_index].xyz;

            const Eigen::Vector3d& n = normals[c.target_index];

            const double residual = n.dot(p - q);

            Eigen::Matrix<double, 1, 6> J;

            J.block<1, 3>(0, 0) = -n.transpose() * skew(p);
            J.block<1, 3>(0, 3) = n.transpose();

            H += J.transpose() * J;
            g += J.transpose() * residual;

            squared_error += residual * residual;
        }

        // Solve H * delta = -g.
        const Eigen::Matrix<double, 6, 1> delta =
            H.ldlt().solve(-g);

        if (!delta.allFinite()) {
            throw std::runtime_error("Point-to-plane optimization failed");
        }

        const Eigen::Vector3d rotation_delta = delta.head<3>();
        const Eigen::Vector3d translation_delta = delta.tail<3>();

        const double rotation_change = rotation_delta.norm();
        const double translation_change = translation_delta.norm();

        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();

        if (rotation_change > 1e-12) {
            delta_R = Eigen::AngleAxisd(
                rotation_change,
                rotation_delta / rotation_change
            ).toRotationMatrix();
        }

        // Apply the incremental left-multiplicative update.
        result.R = delta_R * result.R;
        result.t = delta_R * result.t + translation_delta;

        result.rmse = std::sqrt(
            squared_error /
            static_cast<double>(correspondences.size())
        );

        result.iterations = iteration + 1;
        result.num_correspondences = correspondences.size();

        if (rotation_change < tolerance &&
            translation_change < tolerance) {
            result.converged = true;
            break;
        }
    }

    return result;
}

}