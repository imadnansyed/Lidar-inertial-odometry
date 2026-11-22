#include "../include/lio/registration/icp_convergence.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main()
{
    lio::ConvergenceCriteria criteria;

    criteria.translation_tolerance = 1e-4;
    criteria.rotation_tolerance = 1e-4;
    criteria.rmse_tolerance = 1e-5;
    criteria.minimum_iterations = 3;
    criteria.stable_iterations_required = 2;

    lio::ICPConvergence convergence(criteria);
    lio::ConvergenceState state;

    // Small changes before minimum iteration: must not stop.
    assert(!convergence.shouldStop(state, 1, 1e-6, 1e-6, 0.01));
    assert(!convergence.shouldStop(state, 2, 1e-6, 1e-6, 0.01));

    // First stable iteration after minimum: not enough yet.
    assert(!convergence.shouldStop(state, 3, 1e-6, 1e-6, 0.01));

    // Second stable iteration: stop.
    assert(convergence.shouldStop(state, 4, 1e-6, 1e-6, 0.01));

    // Reset state and test that a large transformation change prevents stopping.
    state = {};

    assert(!convergence.shouldStop(state, 3, 0.1, 0.1, 0.01));
    assert(!convergence.shouldStop(state, 4, 1e-6, 1e-6, 0.01));

    // Invalid measurements.
    bool exception_thrown = false;

    try {
        convergence.shouldStop(state, 5, -1.0, 0.0, 0.01);
    } catch (const std::invalid_argument&) {
        exception_thrown = true;
    }

    assert(exception_thrown);

    std::cout << "All ICP convergence tests passed!\n";

    return 0;
}