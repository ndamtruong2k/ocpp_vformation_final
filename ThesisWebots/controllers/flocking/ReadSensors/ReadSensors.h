#pragma once

#include "webots/DistanceSensor.hpp"
#include "webots/GPS.hpp"
#include "webots/InertialUnit.hpp"
#include "Eigen/Dense"
#include <iostream>
#include <corecrt_math_defines.h>
#include <cmath>
#include <string>


struct sensorValues {
	Eigen::Vector<double, 8> distances;
	Eigen::Matrix<double, 8, 3> coords;
};

class ReadSensors{
private:
	
	webots::DistanceSensor* ds[8];
	double lookup_table[2][2] = { {0, 1000},
								  {1, 0}};

	double sensorsOrientation[8] = { 1.27 - M_PI_2, 0.77 - M_PI_2 ,0.00 - M_PI_2 , 5.21 - M_PI_2, 4.21 - M_PI_2, 3.14159 - M_PI_2, 2.37 - M_PI_2, 1.87 - M_PI_2 };

	Eigen::Matrix<double, 1, 2> lineEquations;

	Eigen::Matrix<double, 1 ,2> findLineEquations();
	Eigen::Matrix<double, 8, 3> dist2Cord(Eigen::Vector<double, 8> sensorsValue);
public:
	void Init(webots::DistanceSensor* ds_[8], int timeStep);
	sensorValues readDistSensors();	
};
