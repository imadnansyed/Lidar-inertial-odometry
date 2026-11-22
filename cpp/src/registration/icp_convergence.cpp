#include "../include/lio/registration/icp_convergence.hpp"

#include <cmath>
#include <stdexcept>

namespace lio {

ICPConvergence::ICPConvergence(
    const ConvergenceCriteria& criteria
)
    : criteria_(criteria)
{
    if (criteria_.translation_tolerance <= 0.0 ||
        criteria_.rotation_tolerance <= 0.0 ||
        criteria_.rmse_tolerance <= 0.0 ||
        !std::isfinite(criteria_.translation_tolerance) ||
        !std::isfinite(criteria_.rotation_tolerance) ||
        !std::isfinite(criteria_.rmse_tolerance) ||
        criteria_.minimum_iterations == 0 ||
        criteria_.stable_iterations_required == 0) {
        throw std::invalid_argument("Invalid ICP convergence criteria");
    }
}

void ICPConvergence::reset()
{
    // This component is stateless between calls.
}

bool ICPConvergence::shouldStop(
    ConvergenceState& state,
    std::size_t iteration,
    double translation_change,
    double rotation_change,
    double current_rmse
) const
{
    if (!std::isfinite(translation_change) ||
        !std::isfinite(rotation_change) ||
        !std::isfinite(current_rmse) ||
        translation_change < 0.0 ||
        rotation_change < 0.0 ||
        current_rmse < 0.0) {
        throw std::invalid_argument("Invalid convergence measurements");
    }

    if (iteration < criteria_.minimum_iterations) {
        state.previous_rmse = current_rmse;
        state.has_previous_rmse = true;
        state.stable_iterations = 0;
        return false;
    }

    const bool transform_stable =
        translation_change < criteria_.translation_tolerance &&
        rotation_change < criteria_.rotation_tolerance;

    bool rmse_stable = false;

    if (state.has_previous_rmse) {
        const double change =
            std::abs(current_rmse - state.previous_rmse);

        rmse_stable =
            change < criteria_.rmse_tolerance;
    }

    state.previous_rmse = current_rmse;
    state.has_previous_rmse = true;

    if (transform_stable && rmse_stable) {
        ++state.stable_iterations;
    } else {
        state.stable_iterations = 0;
    }

    return state.stable_iterations >=
           criteria_.stable_iterations_required;
}

}