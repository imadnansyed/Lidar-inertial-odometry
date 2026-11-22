
#include "../include/lio/registration/nearest_neighbor.hpp"

#include <nanoflann.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace lio {

// --------------------------------------------------
// KD-Tree definition
// --------------------------------------------------

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<
        double,
        NearestNeighbor::PointCloudAdaptor
    >,
    NearestNeighbor::PointCloudAdaptor,
    3,
    std::size_t
>;

// --------------------------------------------------
// Complete Tree definition
// Must appear before search() and searchK()
// --------------------------------------------------

class NearestNeighbor::Tree : public KDTree {
public:
    explicit Tree(const PointCloudAdaptor& adaptor)
        : KDTree(
            3,
            adaptor,
            nanoflann::KDTreeSingleIndexAdaptorParams(10)
        )
    {
        buildIndex();
    }
};

// --------------------------------------------------
// PointCloudAdaptor implementation
// --------------------------------------------------

NearestNeighbor::PointCloudAdaptor::PointCloudAdaptor(
    const PointCloud& cloud
)
    : cloud_(cloud)
{
}

std::size_t
NearestNeighbor::PointCloudAdaptor::kdtree_get_point_count() const
{
    return cloud_.size();
}

double
NearestNeighbor::PointCloudAdaptor::kdtree_get_pt(
    std::size_t index,
    std::size_t dimension
) const
{
    return cloud_[index].xyz[dimension];
}

// --------------------------------------------------
// Constructor
// --------------------------------------------------

NearestNeighbor::NearestNeighbor(const PointCloud& cloud)
    : points_(cloud),
      adaptor_(points_)
{
    if (points_.empty()) {
        throw std::invalid_argument(
            "Cannot build KD-tree from empty point cloud"
        );
    }

    // Validate point cloud coordinates
    for (const auto& point : points_) {
        if (!point.xyz.allFinite()) {
            throw std::invalid_argument(
                "Point cloud contains invalid coordinates"
            );
        }
    }

    tree_ = std::make_unique<Tree>(adaptor_);
}

// --------------------------------------------------
// Destructor
// --------------------------------------------------

NearestNeighbor::~NearestNeighbor() = default;

// --------------------------------------------------
// Single nearest-neighbor search
// --------------------------------------------------

std::optional<NearestNeighbor::NearestNeighborResult>
NearestNeighbor::search(
    const Eigen::Vector3d& query,
    double max_distance
) const
{
    if (!query.allFinite()) {
        throw std::invalid_argument(
            "Query point contains invalid values"
        );
    }

    if (max_distance < 0.0 || std::isnan(max_distance)) {
        throw std::invalid_argument(
            "Invalid maximum search distance"
        );
    }

    std::size_t index = 0;
    double squared_distance = 0.0;

    const double query_point[3] = {
        query.x(),
        query.y(),
        query.z()
    };

    const std::size_t found = tree_->knnSearch(
        query_point,
        1,
        &index,
        &squared_distance
    );

    if (found == 0 ||
        squared_distance > max_distance * max_distance) {
        return std::nullopt;
    }

    return NearestNeighborResult{
        index,
        squared_distance
    };
}

// --------------------------------------------------
// K nearest-neighbor search
// --------------------------------------------------

std::vector<NearestNeighbor::NearestNeighborResult>
NearestNeighbor::searchK(
    const Eigen::Vector3d& query,
    std::size_t k
) const
{
    if (!query.allFinite()) {
        throw std::invalid_argument(
            "Query point contains invalid values"
        );
    }

    if (k == 0) {
        throw std::invalid_argument(
            "k must be greater than zero"
        );
    }

    const std::size_t count = std::min(k, points_.size());

    std::vector<std::size_t> indices(count);
    std::vector<double> distances(count);

    const double query_point[3] = {
        query.x(),
        query.y(),
        query.z()
    };

    const std::size_t found = tree_->knnSearch(
        query_point,
        count,
        indices.data(),
        distances.data()
    );

    std::vector<NearestNeighborResult> results;
    results.reserve(found);

    for (std::size_t i = 0; i < found; ++i) {
        results.push_back({
            indices[i],
            distances[i]
        });
    }

    return results;
}

}
