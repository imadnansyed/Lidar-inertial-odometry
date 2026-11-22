#pragma once

#include "../include/lio/common/types.hpp"
#include "../include/lio/common/se3.hpp"

namespace lio {

class CloudTransform {
public:
    PointCloud transform(
        const PointCloud& input,
        const SE3& T
    ) const;
};

}