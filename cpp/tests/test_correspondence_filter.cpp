#include "../include/lio/registration/correspondence_filter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main()
{
    std::vector<lio::Correspondence> correspondences = {
        {0, 0, 0.010},
        {1, 1, 0.020},
        {2, 2, 0.015},
        {3, 3, 0.030},
        {4, 4, 4.000},  // Outlier
        {5, 5, 0.018}
    };

    // Maximum distance = 1 meter.
    auto filtered = lio::CorrespondenceFilter::filter(
        correspondences,
        1.0,
        3.0
    );

    assert(filtered.size() == 5);

    for (const auto& c : filtered) {
        assert(c.source_index != 4);
        assert(c.squared_distance <= 1.0);
    }

    // Empty input.
    std::vector<lio::Correspondence> empty;

    assert(
        lio::CorrespondenceFilter::filter(empty, 1.0).empty()
    );

    // Invalid settings.
    bool exception_thrown = false;

    try {
        lio::CorrespondenceFilter::filter(
            correspondences,
            -1.0
        );
    } catch (const std::invalid_argument&) {
        exception_thrown = true;
    }

    assert(exception_thrown);

    std::cout << "All correspondence filter tests passed!\n";

    return 0;
}