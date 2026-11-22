#include "../include/lio/mapping/local_mapping.hpp"

#include <stdexcept>

namespace lio {

LocalMapping::LocalMapping(
    const LocalMappingConfig& config
)
    : map_(config.map),
      registration_(map_, config.registration)
{
}

LocalMappingResult LocalMapping::processScan(
    const PointCloud& scan,
    const Eigen::Matrix3d& initial_R,
    const Eigen::Vector3d& initial_t
)
{
    if (!initial_R.allFinite() || !initial_t.allFinite()) {
        throw std::invalid_argument(
            "Invalid initial pose"
        );
    }

    if (scan.empty()) {
        throw std::invalid_argument(
            "Cannot process an empty scan"
        );
    }

    LocalMappingResult result;

    // First scan: initialize the map.
    if (!map_.initialized()) {
        map_.initialize(scan, initial_R, initial_t);

        result.initialized_map = true;
        result.scan_inserted = true;
        result.R = initial_R;
        result.t = initial_t;

        return result;
    }

    // Subsequent scans: align against the existing map.
    result.registration_attempted = true;

    result.registration = registration_.align(
        scan,
        initial_R,
        initial_t
    );

    result.R = result.registration.R;
    result.t = result.registration.t;

    // Reject an unsuccessful or unconverged registration.
    if (!result.registration.success ||
        !result.registration.converged) {
        return result;
    }

    // Insert the scan using its estimated map-frame pose.
    map_.insertScan(
        scan,
        result.registration.R,
        result.registration.t
    );

    result.scan_inserted = true;

    return result;
}

const LocalMap& LocalMapping::map() const
{
    return map_;
}

}