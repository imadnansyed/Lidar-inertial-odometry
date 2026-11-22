#include "../include/lio/io/ground_truth_reader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace lio {

std::vector<PoseRecord> GroundTruthReader::read(
    const std::filesystem::path& file_path
) const
{
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open ground-truth file: " + file_path.string()
        );
    }

    std::vector<PoseRecord> poses;
    std::string line;

    while (std::getline(file, line)) {

        if (line.empty()) {
            continue;
        }

        std::istringstream stream(line);

        Eigen::Matrix<double, 3, 4> matrix;

        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {

                if (!(stream >> matrix(row, col))) {
                    throw std::runtime_error(
                        "Invalid ground-truth pose line: " + line
                    );
                }
            }
        }

        // Reject unexpected extra values
        std::string extra;
        if (stream >> extra) {
            throw std::runtime_error(
                "Unexpected extra values in ground-truth pose: " + line
            );
        }

        PoseRecord pose;

        pose.rotation = matrix.block<3, 3>(0, 0);
        pose.position = matrix.block<3, 1>(0, 3);

        poses.push_back(pose);
    }

    if (poses.empty()) {
        throw std::runtime_error(
            "Ground-truth file contains no poses"
        );
    }

    return poses;
}

} // namespace lio