#pragma once

#include "../include/lio/common/types.hpp"

#include <cstddef>

namespace lio {

class Dataset {
public:
    virtual ~Dataset() = default;

    virtual bool hasNext() const = 0;
    virtual LidarScan next() = 0;
    virtual void reset() = 0;
    virtual std::size_t size() const = 0;
};

}