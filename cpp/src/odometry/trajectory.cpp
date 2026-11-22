#include "../include/lio/odometry/trajectory.hpp"

#include <stdexcept>

namespace lio {

void Trajectory::addPose(
    std::int64_t timestamp_ns,
    const SE3& pose
)
{
    if (!poses_.empty()) {
        if (timestamp_ns <= poses_.back().timestamp_ns) {
            throw std::invalid_argument(
                "Trajectory timestamps must be strictly increasing"
            );
        }
    }

    if (!pose.matrix().allFinite()) {
        throw std::invalid_argument(
            "Cannot add invalid pose to trajectory"
        );
    }

    poses_.push_back({timestamp_ns, pose});
}

const TrajectoryPose& Trajectory::at(std::size_t index) const
{
    return poses_.at(index);
}

const std::vector<TrajectoryPose>& Trajectory::poses() const
{
    return poses_;
}

std::size_t Trajectory::size() const
{
    return poses_.size();
}

bool Trajectory::empty() const
{
    return poses_.empty();
}

void Trajectory::clear()
{
    poses_.clear();
}

}