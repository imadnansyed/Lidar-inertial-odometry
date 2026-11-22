#pragma once

#include "nearest_neighbor.hpp"

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

namespace lio {

struct Correspondence {
    std::size_t source_index;
    std::size_t target_index;
    double squared_distance;
};

class CorrespondenceSearch {
public:
    explicit CorrespondenceSearch(const PointCloud& target);

    std::vector<Correspondence> find(
        const PointCloud& source,
        const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t,
        double max_distance
    ) const;

private:
    NearestNeighbor target_search_;
};

}