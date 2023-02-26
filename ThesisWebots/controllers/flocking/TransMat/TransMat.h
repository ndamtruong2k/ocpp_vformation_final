#pragma once

#include "Eigen/Dense"


Eigen::Matrix3d RotationMat(double roll, double pitch, double yaw);
Eigen::Matrix4d TranslationMat(double roll, double pitch, double yaw, double px, double py, double pz);
Eigen::Matrix<double, 1, 3> Translation(double roll, double pitch, double yaw, double px, double py, double pz, Eigen::Matrix<double,1,3>detectedCoord);
