
#include "TransMat.h"


Eigen::Matrix3d RotationMat(double roll, double pitch, double yaw) {
    Eigen::Matrix3d rotMat;
    rotMat << std::cos(yaw) * std::cos(pitch), (std::cos(yaw) * std::sin(pitch) * std::sin(roll) - std::sin(yaw) * std::cos(roll)), (std::cos(yaw) * std::sin(pitch) * std::cos(roll) + std::sin(yaw) * std::sin(roll)),
        std::sin(yaw)* std::cos(pitch), (std::sin(yaw) * std::sin(pitch) * std::sin(roll) + std::cos(yaw) * std::cos(roll)), (std::sin(yaw) * std::sin(pitch) * std::cos(roll) - std::cos(yaw) * std::sin(roll)),
        -std::sin(pitch), std::cos(pitch)* std::sin(roll), std::cos(pitch)* std::cos(roll);
    return rotMat;
}

Eigen::Matrix4d TranslationMat(double roll, double pitch, double yaw, double px, double py, double pz)
{
    Eigen::Matrix4d transMat;
    transMat << std::cos(yaw) * std::cos(pitch), (std::cos(yaw) * std::sin(pitch) * std::sin(roll) - std::sin(yaw) * std::cos(roll)), (std::cos(yaw) * std::sin(pitch) * std::cos(roll) + std::sin(yaw) * std::sin(roll)), px,
        std::sin(yaw)* std::cos(pitch), (std::sin(yaw) * std::sin(pitch) * std::sin(roll) + std::cos(yaw) * std::cos(roll)), (std::sin(yaw) * std::sin(pitch) * std::cos(roll) - std::cos(yaw) * std::sin(roll)), py,
        -std::sin(pitch), std::cos(pitch)* std::sin(roll), std::cos(pitch)* std::cos(roll), pz,
        0, 0, 0, 1;
    return transMat;
}


Eigen::Matrix<double, 1, 3> Translation(double roll, double pitch, double yaw, double px, double py, double pz, Eigen::Matrix<double, 1, 3>detectedCoord)
{
    Eigen::Matrix<double, 4, 1> temp;
    Eigen::Matrix<double, 4, 1> temp1;
    Eigen::Matrix<double, 1, 3> temp2;
    temp << detectedCoord(0), detectedCoord(1), detectedCoord(2), 1;
    temp1 = TranslationMat(roll, pitch, yaw, px, py, pz) * temp;

    temp2 << temp1(0), temp1(1), temp1(2);

    return temp2;
}