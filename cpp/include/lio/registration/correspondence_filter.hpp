#pragma once

#include "../include/lio/registration/correspondence.hpp"

#include <vector>

namespace lio {

class CorrespondenceFilter {
public:
    static std::vector<Correspondence> filter(
        const std::vector<Correspondence>& correspondences,
        double max_distance,
        double mad_scale = 3.0
    );
};

}