#pragma once

#include <cstddef>

namespace lio {

// LiDAR preprocessing parameters
struct LidarConfig
{
    double min_range = 1.0;       // meters
    double max_range = 100.0;     // meters
    double voxel_size = 0.2;      // meters
};

// ICP registration parameters
struct RegistrationConfig
{
    int max_iterations = 30;

    double max_correspondence_distance = 1.0; // meters
    double transformation_epsilon = 1e-6;
    double fitness_epsilon = 1e-6;

    int min_correspondences = 20;
};

// IMU noise parameters
struct ImuConfig
{
    double gyro_noise_density = 0.001;
    double accel_noise_density = 0.01;

    double gyro_bias_random_walk = 0.0001;
    double accel_bias_random_walk = 0.001;

    double gravity_magnitude = 9.81;
};

// Local mapping parameters
struct MappingConfig
{
    double voxel_size = 0.4;       // meters
    double local_map_radius = 50.0; // meters

    std::size_t max_points_per_voxel = 20;
};

// Main LIO configuration
struct LioConfig
{
    LidarConfig lidar;
    RegistrationConfig registration;
    ImuConfig imu;
    MappingConfig mapping;

    // Validate all configuration parameters
    void validate() const;
};

} // namespace lio