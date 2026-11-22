#include "../include/lio/mapping/local_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lio {

LocalMap::LocalMap(const LocalMapConfig& config)
    : config_(config)
{
    validateConfig();
}

void LocalMap::validateConfig() const
{
    if (config_.max_radius <= 0.0) {
        throw std::invalid_argument(
            "Local map radius must be positive"
        );
    }

    if (config_.max_points == 0) {
        throw std::invalid_argument(
            "Maximum map points must be greater than zero"
        );
    }
}

void LocalMap::initialize(
    const PointCloud& scan,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t
)
{
    if (scan.empty()) {
        throw std::invalid_argument(
            "Cannot initialize local map with an empty scan"
        );
    }

    if (!R.allFinite() || !t.allFinite()) {
        throw std::invalid_argument(
            "Invalid initial map pose"
        );
    }

    clear();

    tree_ = std::make_unique<IKDTree>();

    IKDTree::PointVector tree_points;
    tree_points.reserve(scan.size());

    points_.reserve(scan.size());

    for (const auto& point : scan) {
        if (!point.xyz.allFinite()) {
            continue;
        }

        const Eigen::Vector3d map_point =
            R * point.xyz + t;

        points_.push_back({
            map_point,
            point.intensity
        });

        tree_points.emplace_back(
            static_cast<float>(map_point.x()),
            static_cast<float>(map_point.y()),
            static_cast<float>(map_point.z())
        );
    }

    if (points_.empty()) {
        clear();

        throw std::invalid_argument(
            "Initial scan contains no valid points"
        );
    }

    tree_->Build(std::move(tree_points));

    center_ = t;
    initialized_ = true;

    prune(center_);
}

void LocalMap::insertScan(
    const PointCloud& scan,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t
)
{
    if (!initialized_ || !tree_) {
        throw std::runtime_error(
            "Local map must be initialized before insertion"
        );
    }

    if (!R.allFinite() || !t.allFinite()) {
        throw std::invalid_argument(
            "Invalid scan pose"
        );
    }

    center_ = t;

    IKDTree::PointVector new_points;
    new_points.reserve(scan.size());

    for (const auto& point : scan) {
        if (!point.xyz.allFinite()) {
            continue;
        }

        const Eigen::Vector3d map_point =
            R * point.xyz + t;

        points_.push_back({
            map_point,
            point.intensity
        });

        new_points.emplace_back(
            static_cast<float>(map_point.x()),
            static_cast<float>(map_point.y()),
            static_cast<float>(map_point.z())
        );
    }

    if (!new_points.empty()) {
        tree_->Add_Points(new_points, false);
    }

    prune(center_);
}

void LocalMap::prune(const Eigen::Vector3d& center)
{
    if (!center.allFinite()) {
        throw std::invalid_argument(
            "Invalid local map center"
        );
    }

    if (!initialized_ || !tree_) {
        return;
    }

    const double radius_squared =
        config_.max_radius * config_.max_radius;

    PointCloud kept_points;
    IKDTree::PointVector removed_points;

    kept_points.reserve(points_.size());

    // 1. Remove points outside the spherical local-map radius.
    for (const auto& point : points_) {
        const double distance_squared =
            (point.xyz - center).squaredNorm();

        if (distance_squared > radius_squared) {
            removed_points.emplace_back(
                static_cast<float>(point.xyz.x()),
                static_cast<float>(point.xyz.y()),
                static_cast<float>(point.xyz.z())
            );
        } else {
            kept_points.push_back(point);
        }
    }

    // 2. Enforce the maximum point count.
    if (kept_points.size() > config_.max_points) {
        const std::size_t excess =
            kept_points.size() - config_.max_points;

        for (std::size_t i = 0; i < excess; ++i) {
            removed_points.emplace_back(
                static_cast<float>(kept_points[i].xyz.x()),
                static_cast<float>(kept_points[i].xyz.y()),
                static_cast<float>(kept_points[i].xyz.z())
            );
        }

        kept_points.erase(
            kept_points.begin(),
            kept_points.begin() + excess
        );
    }

    // 3. Delete the same points from iKD-Tree.
    if (!removed_points.empty()) {
        tree_->Delete_Points(removed_points);
    }

    points_.swap(kept_points);
    center_ = center;
}

std::vector<MapNeighbor> LocalMap::nearestKSearch(
    const Eigen::Vector3d& query,
    std::size_t k,
    double max_distance
) const
{
    if (!query.allFinite()) {
        throw std::invalid_argument(
            "Invalid nearest-neighbor query"
        );
    }

    if (max_distance <= 0.0) {
        throw std::invalid_argument(
            "Maximum search distance must be positive"
        );
    }

    std::vector<MapNeighbor> neighbors;

    if (!initialized_ || !tree_ || k == 0 || empty()) {
        return neighbors;
    }

    const std::size_t search_count =
        std::min({
            k,
            points_.size(),
            static_cast<std::size_t>(
                std::numeric_limits<int>::max()
            )
        });

    IKDTree::PointVector nearest_points;
    std::vector<float> distances;

    const ikdTree_PointType query_point(
        static_cast<float>(query.x()),
        static_cast<float>(query.y()),
        static_cast<float>(query.z())
    );

    tree_->Nearest_Search(
        query_point,
        static_cast<int>(search_count),
        nearest_points,
        distances,
        max_distance
    );

    neighbors.reserve(nearest_points.size());

    for (std::size_t i = 0; i < nearest_points.size(); ++i) {
        const auto& point = nearest_points[i];

        neighbors.push_back({
            Eigen::Vector3d(
                point.x,
                point.y,
                point.z
            ),
            static_cast<double>(distances[i])
        });
    }

    return neighbors;
}

const PointCloud& LocalMap::points() const
{
    return points_;
}

std::size_t LocalMap::size() const
{
    return points_.size();
}

bool LocalMap::empty() const
{
    return points_.empty();
}

bool LocalMap::initialized() const
{
    return initialized_;
}

void LocalMap::clear()
{
    // Destroying the tree also stops its background rebuild thread.
    tree_.reset();

    points_.clear();
    center_.setZero();
    initialized_ = false;
}

}