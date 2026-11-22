#pragma once

#include <cstddef>

namespace lio {

struct ConvergenceState {
    double previous_rmse = 0.0;
    std::size_t stable_iterations = 0;
    bool has_previous_rmse = false;
};

struct ConvergenceCriteria {
    double translation_tolerance = 1e-6;
    double rotation_tolerance = 1e-6;
    double rmse_tolerance = 1e-8;
    std::size_t minimum_iterations = 3;
    std::size_t stable_iterations_required = 2;
};

class ICPConvergence {
public:
    explicit ICPConvergence(
        const ConvergenceCriteria& criteria = {}
    );

    void reset();

    bool shouldStop(
        ConvergenceState& state,
        std::size_t iteration,
        double translation_change,
        double rotation_change,
        double current_rmse
    ) const;

private:
    ConvergenceCriteria criteria_;
};

}