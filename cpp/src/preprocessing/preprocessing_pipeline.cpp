#include "../include/lio/preprocessing/preprocessing_pipeline.hpp"

namespace lio {

PreprocessingPipeline::PreprocessingPipeline(
    const LioConfig& config
)
    : range_filter_(config.lidar),
      voxel_filter_(config.lidar.voxel_size)
{
}

LidarScan PreprocessingPipeline::process(
    const LidarScan& scan
) const
{
    LidarScan output;

    // Preserve scan timestamp
    output.timestamp_ns = scan.timestamp_ns;

    // Step 1: Remove invalid points
    PointCloud valid_points =
        invalid_filter_.filter(scan.points);

    // Step 2: Apply minimum and maximum range
    PointCloud range_filtered =
        range_filter_.filter(valid_points);

    // Step 3: Downsample
    output.points =
        voxel_filter_.filter(range_filtered);

    return output;
}

}