#pragma once

#include "../include/lio/common/types.hpp"

namespace lio {

class InvalidPointFilter {
public:
    PointCloud filter(const PointCloud& input) const;
};

}