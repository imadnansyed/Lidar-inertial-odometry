#pragma once

#include "../include/lio/common/se3.hpp"
#include "relative_pose.hpp"

#include <vector>

namespace lio {

class PoseAccumulator {
public:
    PoseAccumulator();

    void reset(const SE3& initial_pose = SE3::Identity());

    SE3 update(const RelativePoseEstimate& relative_pose);

    const SE3& currentPose() const;

    const std::vector<SE3>& poses() const;

    std::size_t size() const;

private:
    SE3 current_pose_;

    std::vector<SE3> poses_;
};

}