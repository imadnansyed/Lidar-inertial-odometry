#include "../include/lio/common/config.hpp"

#include <stdexcept>
#include <string>

namespace lio {

void LioConfig::validate() const
{
    // LiDAR validation
    if (lidar.min_range < 0.0)
        throw std::invalid_argument("LiDAR minimum range cannot be negative");

    if (lidar.max_range <= lidar.min_range)
        throw std::invalid_argument("LiDAR maximum range must exceed minimum range");

    if (lidar.voxel_size <= 0.0)
        throw std::invalid_argument("LiDAR voxel size must be positive");

    // Registration validation
    if (registration.max_iterations <= 0)
        throw std::invalid_argument("ICP maximum iterations must be positive");

    if (registration.max_correspondence_distance <= 0.0)
        throw std::invalid_argument("ICP correspondence distance must be positive");

    if (registration.transformation_epsilon < 0.0)
        throw std::invalid_argument("ICP transformation epsilon cannot be negative");

    if (registration.fitness_epsilon < 0.0)
        throw std::invalid_argument("ICP fitness epsilon cannot be negative");

    if (registration.min_correspondences < 3)
        throw std::invalid_argument("ICP needs at least 3 correspondences");

    // IMU validation
    if (imu.gyro_noise_density <= 0.0)
        throw std::invalid_argument("Gyroscope noise density must be positive");

    if (imu.accel_noise_density <= 0.0)
        throw std::invalid_argument("Accelerometer noise density must be positive");

    if (imu.gyro_bias_random_walk < 0.0)
        throw std::invalid_argument("Gyroscope bias random walk cannot be negative");

    if (imu.accel_bias_random_walk < 0.0)
        throw std::invalid_argument("Accelerometer bias random walk cannot be negative");

    if (imu.gravity_magnitude <= 0.0)
        throw std::invalid_argument("Gravity magnitude must be positive");

    // Mapping validation
    if (mapping.voxel_size <= 0.0)
        throw std::invalid_argument("Map voxel size must be positive");

    if (mapping.local_map_radius <= 0.0)
        throw std::invalid_argument("Local map radius must be positive");

    if (mapping.max_points_per_voxel == 0)
        throw std::invalid_argument("Maximum points per voxel must be positive");
}

} // namespace lio