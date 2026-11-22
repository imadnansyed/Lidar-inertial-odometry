#pragma once

#include "../include/lio/common/types.hpp"

#include <ikd_Tree.h>

#include <Eigen/Dense>

#include <cstddef>
#include <memory>
#include <vector>

namespace lio {

struct LocalMapConfig {
    double max_radius = 20.0;
    std::size_t max_points = 200000;
};

struct MapNeighbor {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    double squared_distance = 0.0;
};

class LocalMap {
public:
    explicit LocalMap(
        const LocalMapConfig& config = LocalMapConfig{}
    );

    void initialize(
        const PointCloud& scan,
        const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity(),
        const Eigen::Vector3d& t = Eigen::Vector3d::Zero()
    );

    void insertScan(
        const PointCloud& scan,
        const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t
    );

    void prune(const Eigen::Vector3d& center);

    std::vector<MapNeighbor> nearestKSearch(
        const Eigen::Vector3d& query,
        std::size_t k,
        double max_distance
    ) const;

    const PointCloud& points() const;
    std::size_t size() const;
    bool empty() const;
    bool initialized() const;
    void clear();

private:
    using IKDTree = KD_TREE<ikdTree_PointType>;

    LocalMapConfig config_;
    PointCloud points_;

    std::unique_ptr<IKDTree> tree_;

    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    bool initialized_ = false;

    void validateConfig() const;
};

}