
#pragma once

#include "../include/lio/common/types.hpp"

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace lio {

class NearestNeighbor {
public:
    struct NearestNeighborResult {
        std::size_t index;
        double squared_distance;
    };

    class PointCloudAdaptor {
    public:
        explicit PointCloudAdaptor(const PointCloud& cloud);

        std::size_t kdtree_get_point_count() const;

        double kdtree_get_pt(
            std::size_t index,
            std::size_t dimension
        ) const;

        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const {
            return false;
        }

    private:
        const PointCloud& cloud_;
    };

    explicit NearestNeighbor(const PointCloud& cloud);

    ~NearestNeighbor();

    std::optional<NearestNeighborResult> search(
        const Eigen::Vector3d& query,
        double max_distance =
            std::numeric_limits<double>::infinity()
    ) const;

    std::vector<NearestNeighborResult> searchK(
        const Eigen::Vector3d& query,
        std::size_t k
    ) const;

private:
    PointCloud points_;
    PointCloudAdaptor adaptor_;

    class Tree;
    std::unique_ptr<Tree> tree_;
};

}
