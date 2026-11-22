#pragma once

#include "local_map.hpp"
#include "../include/lio/registration/scan_to_map_registration.hpp"

namespace lio {

struct LocalMappingConfig {
    LocalMapConfig map;
    ScanToMapConfig registration;
};

struct LocalMappingResult {
    bool initialized_map = false;
    bool registration_attempted = false;
    bool scan_inserted = false;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    ScanToMapResult registration;
};

class LocalMapping {
public:
    explicit LocalMapping(
        const LocalMappingConfig& config = LocalMappingConfig{}
    );

    LocalMappingResult processScan(
        const PointCloud& scan,
        const Eigen::Matrix3d& initial_R = Eigen::Matrix3d::Identity(),
        const Eigen::Vector3d& initial_t = Eigen::Vector3d::Zero()
    );

    const LocalMap& map() const;

private:
    // Declaration order matters: registration_ references map_.
    LocalMap map_;
    ScanToMapRegistration registration_;
};

}