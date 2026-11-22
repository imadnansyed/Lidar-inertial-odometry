#pragma once

#include "../include/lio/common/se3.hpp"

#include <cstdint>
#include <vector>
#include <cstddef>

namespace lio {

struct TrajectoryPose {
    std::int64_t timestamp_ns = 0;
    SE3 pose = SE3::Identity();
};

class Trajectory {
public:
    void addPose(
        std::int64_t timestamp_ns,
        const SE3& pose
    );

    const TrajectoryPose& at(std::size_t index) const;

    const std::vector<TrajectoryPose>& poses() const;

    std::size_t size() const;

    bool empty() const;

    void clear();

private:
    std::vector<TrajectoryPose> poses_;
};

}