#include <iostream>
#include <cassert>
#include <stdexcept>

#include "../include/lio/common/config.hpp"

int main()
{
    using namespace lio;

    // Test 1: Default configuration
    LioConfig config;

    config.validate();

    assert(config.lidar.voxel_size == 0.2);
    assert(config.registration.max_iterations == 30);
    assert(config.imu.gravity_magnitude == 9.81);

    // Test 2: Change parameters
    config.lidar.voxel_size = 0.3;
    config.registration.max_iterations = 50;

    config.validate();

    assert(config.lidar.voxel_size == 0.3);
    assert(config.registration.max_iterations == 50);

    // Test 3: Invalid voxel size
    config.lidar.voxel_size = -0.1;

    bool exception_caught = false;

    try
    {
        config.validate();
    }
    catch (const std::invalid_argument&)
    {
        exception_caught = true;
    }

    assert(exception_caught);

    // Test 4: Invalid range
    config = LioConfig{};
    config.lidar.min_range = 100.0;
    config.lidar.max_range = 50.0;

    exception_caught = false;

    try
    {
        config.validate();
    }
    catch (const std::invalid_argument&)
    {
        exception_caught = true;
    }

    assert(exception_caught);

    std::cout << "All configuration tests passed!\n";

    return 0;
}