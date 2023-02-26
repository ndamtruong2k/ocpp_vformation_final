// File:          coverage.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Compass.hpp>
#include <Eigen/Dense>
#include <string>
#include <cmath>
#include <fstream> 
#include "ReadSensors.h"
#include "TransMat.h"
#include "BehaviorControl.h"

// All the webots classes are defined in the "webots" namespace

using namespace webots;


Eigen::Matrix3d RotationMat(double roll, double pitch, double yaw);


//Hàm tham số điều khiển
const double DIS_COHENSION = 0.1;

struct infoRobot {
    std::string robotName;
    double oretationRobot;
    double velocityRobot;
    int state_nei;
    bool doneTarget = false;
};

double Distance(Eigen::Matrix<double, 1, 3> position1, Eigen::Matrix<double, 1, 3> position2) {
    return sqrt(pow(position1(0) - position2(0), 2) + pow(position1(1) - position2(1), 2) + pow(position1(2) - position2(2), 2));
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  int id = robot->getNumberOfDevices();

  std::ofstream positionFile;
  std::string name = robot->getName();
  std::cout << robot->getName() << std::endl;

  positionFile.open(name + "_testEntro.txt");
  std::cout << "Tao file xong" << std::endl;

  GPS* gps = robot->getGPS("gps");
  InertialUnit* imu = robot->getInertialUnit("inertial unit");
  Emitter* emitter = robot->getEmitter("emitter");
  Receiver* receiver = robot->getReceiver("receiver");
  Accelerometer* accelerometer = robot->getAccelerometer("accelerometer");
  Compass* cpass = robot->getCompass("compass");
  Motor* left_motor = robot->getMotor("left wheel motor");
  Motor* right_motor = robot->getMotor("right wheel motor");
  DistanceSensor* ds[8];
  for (int i = 0; i < 8; i++)
  {
      ds[i] = robot->getDistanceSensor("ps" + std::to_string(i));
      ds[i]->enable(timeStep);
  }
  gps->enable(timeStep);
  imu->enable(timeStep);
  receiver->enable(timeStep);
  accelerometer->enable(timeStep);
  cpass->enable(timeStep);

  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);
  /*emitter->setRange(0.8);*/


  BehaviorControl bc(left_motor, right_motor, gps, imu, ds,emitter,receiver, timeStep);

  ReadSensors RS;
  RS.Init(ds,timeStep);

  // Khởi tạo hàng xóm
  bool robotNei = false;
  int numberNei = 0;

  // Main loop:
  //Initialize the counter
  int i;
  // Information of Robot
  infoRobot info;
  const infoRobot* dataRecei;

  int k = 0;
  int soLuong = 0;

  double disTarget;
  int time_count = 0;

  while (robot->step(timeStep) != -1) {
      
      std::cout << "-----------------------------------------------" << std::endl;
      // Tên của robot
      std::cout << "RobotID: " << robot->getName() << std::endl;
      /*for (int i = 0; i < 8; i++)
      {
          double value = ds[i]->getValue();
          std::cout << "Cam bien thu " << i + 1 << " :" << RS.readDistSensors().distances(i) << std::endl;
      }
      std::cout << "*******************" << std::endl;*/
      Eigen::Matrix<double, 1, 3> goal;
      goal << 3, gps->getValues()[1], 0;
      if (time_count < 4000) {
          goal << 2,gps->getValues()[1], 0;   
      }
      else if (time_count >= 4000 && time_count < 8000) {
          goal << gps->getValues()[0], 2, 0;
      }
      else if (time_count >= 8000 && time_count < 10000) {
          goal << -2, gps->getValues()[1], 0;
      }
      else if (time_count >= 10000 && time_count < 12000) {
          goal << gps->getValues()[0], -2, 0;
      }
      else{
          goal << gps->getValues()[0], gps->getValues()[1], 0;
      }
      bc.BC(goal);

      Eigen::Matrix<double, 1, 3> position;
      position << gps->getValues()[0], gps->getValues()[1], gps->getValues()[2];
      disTarget = Distance(goal, position);

      std::cout << "Cap nhat: Status -  " << bc.status << std::endl;
      // Cập nhật và gửi thông tin;
      info.robotName = robot->getName();
      info.oretationRobot = imu->getRollPitchYaw()[2];
      info.velocityRobot = bc.velocityRobot;
      info.state_nei = bc.status;
      /*info.status = bc.*/
      if (disTarget < 0.1) {
          info.doneTarget = true;
      }
      
      emitter->send(&info, sizeof(info));

      positionFile << gps->getValues()[0] << " " << gps->getValues()[1] << " " << gps->getValues()[2] << " " << imu->getRollPitchYaw()[2] << " " << bc.errorAngle << "\n";
      std::cout << "Time: " << time_count << std::endl;

      time_count++;

  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}