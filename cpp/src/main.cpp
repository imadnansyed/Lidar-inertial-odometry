#include <iostream>
#include <Eigen/Dense>

int main()
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    Eigen::Vector3d t(1.0, 2.0, 3.0);

    std::cout << "Rotation:\n" << R << "\n";
    std::cout << "Translation:\n" << t << "\n";

    return 0;
}