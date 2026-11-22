#include "../include/lio/preprocessing/cloud_transform.hpp"

namespace lio {

PointCloud CloudTransform::transform(
    const PointCloud& input,
    const SE3& T
) const
{
    PointCloud output;
    output.reserve(input.size());

    for (const auto& point : input) {
        PointXYZI transformed_point;

        transformed_point.xyz = T * point.xyz;
        transformed_point.intensity = point.intensity;

        output.push_back(transformed_point);
    }

    return output;
}

}