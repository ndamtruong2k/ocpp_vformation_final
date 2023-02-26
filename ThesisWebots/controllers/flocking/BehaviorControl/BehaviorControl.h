#pragma once

#include "webots/GPS.hpp"
#include "webots/Motor.hpp"
#include "webots/InertialUnit.hpp"
#include "webots/DistanceSensor.hpp"
#include "webots/Emitter.hpp"
#include "webots/Receiver.hpp"
#include "Eigen/Dense"
#include "TransMat.h"
#include "ReadSensors.h"
#include <cmath>
#include <queue>
#include <string>




using namespace webots;

class BehaviorControl {
private:
	//Hàm tham số điều khiển
	const double DIS_COHENSION = 0.5;

	double R = 0.0205;
	double L = 0.052;

	double d_m;
	double a_m = 1.0;
	double b_m = 0.1;

	double d_0;
	double a_0 = 5.0;
	double b_0 = 0.1;
	double b_f = 0.0015;
	
	double a_f = 5.0;

	double obsOrientation;

	double d_c;
	double a_c = 5.0;
	double b_c = 0.7;
	double b_co = 0.5;

	double d_a;

	double o_init;// vecto lam cho moi robot cung huong ban dau;

	std::queue<double> errorQueue;
	double Kp = 300;
	double Ki = 150;


	//double Kp = 1200;
	//double Ki = 0;

	Motor* leftMotor;
	Motor* rightMotor;
	GPS* gps;
	InertialUnit* imu;
	Emitter* emitter;
	Receiver* receiver;


	ReadSensors RS;

	Eigen::Matrix<double, 1, 3> move2goal(Eigen::Matrix<double, 1, 3> goal_);
	Eigen::Matrix<double, 1, 3> obstacleAvoidance();
	Eigen::Matrix<double, 1, 3> cohesion();
	Eigen::Matrix<double, 1, 3> alignment();

	double F0();
	double F1();
	double F2();
	double F3();
	
	double Distance(Eigen::Matrix<double, 1, 3> position1, Eigen::Matrix<double, 1, 3> position2);


	struct infoRobot {
		std::string robotName;
		double oretationRobot;
		double VelocityRobot;
		int state_nei;
		bool doneTarget = false;
	};

	//Initialize the counter
	int i;
	// Information of Robot
	int soLuong;
	infoRobot info;
	const infoRobot* dataRecei;
public:
	BehaviorControl(Motor* leftMotor_, Motor* rightMotor_, GPS* gps_, InertialUnit* imu_, DistanceSensor* ds_[8], Emitter* emitter,
	Receiver* receiver, int timeStep_);
	double velocityRobot;
	int status;
	double errorAngle;
	void BC(Eigen::Matrix<double, 1, 3> goal_);
};
