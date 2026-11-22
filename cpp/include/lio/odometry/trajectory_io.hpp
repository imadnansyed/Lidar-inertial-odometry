#pragma once

#include "trajectory.hpp"

#include <string>

namespace lio {

class TrajectoryWriter {
public:
    static void writeKITTI(
        const Trajectory& trajectory,
        const std::string& filepath
    );

    static void writeTUM(
        const Trajectory& trajectory,
        const std::string& filepath
    );
};

}