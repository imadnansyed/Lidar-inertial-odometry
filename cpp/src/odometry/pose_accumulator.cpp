#include "../include/lio/odometry/pose_accumulator.hpp"

namespace lio {

PoseAccumulator::PoseAccumulator()
{
    reset();
}

void PoseAccumulator::reset(const SE3& initial_pose)
{
    current_pose_ = initial_pose;

    poses_.clear();
    poses_.push_back(current_pose_);
}

SE3 PoseAccumulator::update(
    const RelativePoseEstimate& relative_pose
)
{
    // ICP estimates source -> target.
    // Invert it for trajectory accumulation.
    const SE3 T_next_current(
        relative_pose.R,
        relative_pose.t
    );

    const SE3 T_current_next = T_next_current.inverse();

    current_pose_ = current_pose_ * T_current_next;

    poses_.push_back(current_pose_);

    return current_pose_;
}

const SE3& PoseAccumulator::currentPose() const
{
    return current_pose_;
}

const std::vector<SE3>& PoseAccumulator::poses() const
{
    return poses_;
}

std::size_t PoseAccumulator::size() const
{
    return poses_.size();
}

}