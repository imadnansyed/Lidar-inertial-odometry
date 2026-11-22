#include "../include/lio/io/kitti_reader.hpp"

#include <fstream>
#include <stdexcept>
#include <vector>
#include <cmath>

namespace lio {

LidarScan KittiReader::readVelodyneScan(
    const std::filesystem::path& file_path,
    int64_t timestamp_ns
) const
{
    std::ifstream file(file_path, std::ios::binary);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open KITTI file: " + file_path.string()
        );
    }

    file.seekg(0, std::ios::end);
    const std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    constexpr std::size_t values_per_point = 4;
    constexpr std::size_t bytes_per_value = sizeof(float);
    constexpr std::size_t bytes_per_point =
        values_per_point * bytes_per_value;

    if (file_size < 0 ||
        file_size % bytes_per_point != 0) {
        throw std::runtime_error(
            "Invalid KITTI Velodyne file size"
        );
    }

    const std::size_t num_points =
        static_cast<std::size_t>(file_size) / bytes_per_point;

    std::vector<float> raw_data(
        num_points * values_per_point
    );

    if (!raw_data.empty()) {
        file.read(
            reinterpret_cast<char*>(raw_data.data()),
            file_size
        );

        if (!file) {
            throw std::runtime_error(
                "Failed to read KITTI Velodyne data"
            );
        }
    }

    LidarScan scan;
    scan.timestamp_ns = timestamp_ns;
    scan.points.reserve(num_points);

    for (std::size_t i = 0; i < raw_data.size(); i += 4) {

        const float x = raw_data[i];
        const float y = raw_data[i + 1];
        const float z = raw_data[i + 2];
        const float intensity = raw_data[i + 3];

        if (!std::isfinite(x) ||
            !std::isfinite(y) ||
            !std::isfinite(z) ||
            !std::isfinite(intensity)) {
            continue;
        }

        PointXYZI point;

        point.xyz = Eigen::Vector3d(x, y, z);
        point.intensity = intensity;

        scan.points.push_back(point);
    }

    return scan;
}

} // namespace lio