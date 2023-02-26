#include "BehaviorControl.h"



BehaviorControl::BehaviorControl(Motor* leftMotor_, Motor* rightMotor_, GPS* gps_, InertialUnit* imu_, DistanceSensor* ds_[8], Emitter* emitter,
	Receiver* receiver_, int timeStep_){
	leftMotor = leftMotor_;
	rightMotor = rightMotor_;
	gps = gps_;
	imu = imu_;
	receiver = receiver_;

	RS.Init(ds_,timeStep_);
	gps->enable(timeStep_);
	imu->enable(timeStep_);
	receiver->enable(timeStep_);



	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
	leftMotor->setVelocity(0.0);
	rightMotor->setVelocity(0.0);
	d_0 = 0;
	d_m = 0;
	d_c = 0;
	d_a = 0;
	o_init = 0;

}

double BehaviorControl::Distance(Eigen::Matrix<double, 1, 3> position1, Eigen::Matrix<double, 1, 3> position2) {
	return sqrt(pow(position1(0) - position2(0), 2) + pow(position1(1) - position2(1), 2) + pow(position1(2) - position2(2), 2));
}

Eigen::Matrix<double, 1, 3> BehaviorControl::move2goal(Eigen::Matrix<double, 1, 3> goal_)
{
	Eigen::Matrix<double, 1, 3> V_m2g;
	Eigen::Matrix<double, 1, 3> currentPosition;
	currentPosition << gps->getValues()[0], gps->getValues()[1], gps->getValues()[2];

	

	V_m2g = (goal_ - currentPosition);
	V_m2g(2) = 0;

	d_m = V_m2g.norm();
	V_m2g *= (1 / d_m);


	std::cout << "Move to goal: " << V_m2g << std::endl;
	return V_m2g;
}

//

Eigen::Matrix<double, 1, 3> BehaviorControl::obstacleAvoidance()
{
	sensorValues sensorsValue = RS.readDistSensors();
	Eigen::Matrix<double, 1, 3> V_obs;
	V_obs << 0, 0, 0;

	int minRow, minCol;
	double minValue = sensorsValue.distances.minCoeff(&minRow, &minCol);


	Eigen::Matrix<double, 1, 3> orientation;
	Eigen::Matrix<double, 1, 3> position;
	orientation << imu->getRollPitchYaw()[0], imu->getRollPitchYaw()[1], imu->getRollPitchYaw()[2];
	position << gps->getValues()[0], gps->getValues()[1], gps->getValues()[2];

	Eigen::Matrix<double, 1, 3> detectedCoord = Translation(orientation(0), orientation(1), orientation(2),
		position(0), position(1), position(2), sensorsValue.coords.row(minRow));
	Eigen::Matrix<double, 1, 3> temp1 = detectedCoord - position;
	temp1(2) = 0;
	d_0 = temp1.norm();

	double angle1 = atan2(0.0, 1.0);
	double angle2 = atan2(temp1(1), temp1(0));
	obsOrientation = angle2 - angle1;
	double robotYaw = orientation(2);
	Eigen::Matrix<double, 1, 3> temp2;


	if (obsOrientation > robotYaw)
	{
		//right
		temp2 << -temp1(0), -1 * temp1(1), temp1(2);
	}
	else
	{
		//left
		temp2 << -1 * temp1(0), -temp1(1), temp1(2);
	}

	V_obs = temp2;
	V_obs *= (1 / d_0);


	//std::cout << "minRow:" << "\n";
	//std::cout << minRow << "\n";

	//std::cout << "obsOrientation:" << "\n";
	//std::cout << obsOrientation << "\n";

	//std::cout << "robotYaw:" << "\n";
	//std::cout << robotYaw << "\n";

	//std::cout << "temp1:" << "\n";
	//std::cout << temp1 << "\n";

	//std::cout << "temp2:" << "\n";
	//std::cout << temp2 << "\n";

	////std::cout << "Distances:" << "\n";
	////std::cout << sensorsValue.distances << "\n";
	//std::cout << "-------------------------------------------------" << "\n";

	return V_obs;
}


Eigen::Matrix<double, 1, 3> BehaviorControl::alignment() {
	// Số lượng robot nhận thông tin
	soLuong = receiver->getQueueLength() - soLuong;
	
	//Heading alignment
	Eigen::Matrix<double, 1, 3> H_alig;
	H_alig << 0, 0, 0;

	Eigen::Matrix<double, 1, 3> temp1;
	temp1 << 0, 0, 0;


	// Lấy dữ liệu data vào trong một lần và chạy. Khi chạy hết data sẽ bắt đầu tiếp nhận thông tin mới
	while(receiver->getQueueLength() > 0) {
		int sizeData = receiver->getDataSize();
		dataRecei = (infoRobot*)receiver->getData();
		infoRobot data_Recei = *dataRecei;
		
		//Heading sum;
		temp1(0) = temp1(0) + data_Recei.VelocityRobot * cos(data_Recei.oretationRobot);
		temp1(1) = temp1(1) + data_Recei.VelocityRobot * sin(data_Recei.oretationRobot);

		
		receiver->nextPacket();
	}
	temp1(2) = 0;
	d_a = temp1.norm();
	if (d_a != 0) {
		H_alig << temp1(1), temp1(0), temp1(2);
		H_alig *= (1 / d_a);
	}else {
		H_alig << 0, 0, 0;
	}
	std::cout << "Alignment: " << H_alig << std::endl;
	return H_alig;

}

Eigen::Matrix<double, 1, 3> BehaviorControl::cohesion() 
{
	sensorValues sensorsValue = RS.readDistSensors();

	Eigen::Matrix<double, 1, 3> V_cohes;

	V_cohes << 0, 0, 0;

	Eigen::Matrix<double, 1, 3> orientation;
	Eigen::Matrix<double, 1, 3> position;
	orientation << imu->getRollPitchYaw()[0], imu->getRollPitchYaw()[1], imu->getRollPitchYaw()[2];
	position << gps->getValues()[0], gps->getValues()[1], gps->getValues()[2];

	Eigen::Matrix<double, 8, 3> detectedCoord;
	Eigen::Matrix<double, 8, 3> temp1;
	Eigen::Matrix<double, 1, 3> temp_sum;

	temp_sum << 0, 0, 0;

	for (i = 0; i < 8; i++) {
		detectedCoord.row(i) = Translation(orientation(0), orientation(1), orientation(2),
			position(0), position(1), position(2), sensorsValue.coords.row(i));
		temp1.row(i) = detectedCoord.row(i) - position;
		temp1.row(i)(2) = 0;

		if (temp1.row(i).norm() >= 0.99) {
			temp1.row(i) << 0, 0, 0;
		}
		temp_sum += temp1.row(i);

	}

	d_c = temp_sum.norm();

	Eigen::Matrix<double, 1, 3> temp2;
	if (d_c != 0) {

		double angle1 = atan2(0.0, 1.0);
		double angle2 = atan2(temp_sum(1), temp_sum(0));
		obsOrientation = angle2 - angle1;
		double robotYaw = orientation(2);


		if (obsOrientation > robotYaw)
		{
			//right
			temp2 << temp_sum(0), 1 * temp_sum(1), temp_sum(2);
		}
		else
		{
			//left
			temp2 << 1 * temp_sum(0), temp_sum(1), temp_sum(2);
		}
		temp2 *= (1 / d_c);
		V_cohes += temp2;
	}
	else {
		V_cohes << 0, 0, 0;

	}

	std::cout << "Cohesion: " << V_cohes << std::endl;
	return V_cohes;
}

double BehaviorControl::F0()
{
	if (d_m > b_m)
	{
		return a_m;
	}
	else
	{
		return a_m * (d_m / b_m);
	}
}

double BehaviorControl::F1()
{
	if ((d_0 >= b_f) && (d_0 <= b_0))
	{
		double f2 = a_0 * (d_0 / (b_f - b_0) + b_0 / (b_0 - b_f));
		return f2;
	}
	else
	{
		if (d_0 > b_0)
		{
			return 0.0;
		}
		else
		{
			return a_f;
		}
	}
}

double BehaviorControl::F2()
{
	if ((d_c >= b_co) && (d_c <= b_c))
	{
		double f2 = a_c * (d_c / (b_co - b_c) + b_c / (b_c - b_co));
		return f2;
	}
	else
	{
		if (d_c > b_c)
		{
			return a_f;
		}
		else
		{
			return 0.0;
		}
	}
}

void BehaviorControl::BC(Eigen::Matrix<double, 1, 3> goal_)
{
	int nei = 1;

	// Ban đầu khởi tạo ở trạng thái mất kết nối giả
	status = 0;

	Eigen::Matrix<double, 1, 3> V;
	Eigen::Matrix<double, 1, 3> V_m2g = move2goal(goal_);
	Eigen::Matrix<double, 1, 3> V_obs = obstacleAvoidance();
	Eigen::Matrix<double, 1, 3> V_coh = cohesion();
	/*Eigen::Matrix<double, 1, 3> V_ali = alignment();*/

	sensorValues sensorsValue = RS.readDistSensors();
	Eigen::Matrix<double, 1, 3> orientation;
	Eigen::Matrix<double, 1, 3> position;
	orientation << imu->getRollPitchYaw()[0], imu->getRollPitchYaw()[1], imu->getRollPitchYaw()[2];
	position << gps->getValues()[0], gps->getValues()[1], gps->getValues()[2];

	Eigen::Matrix<double, 8, 3> detectedCoord;
	Eigen::Matrix<double, 8, 3> temp1;
	Eigen::Matrix<double, 1, 3> temp_sum;

	for (i = 0; i < 8; i++) {
		detectedCoord.row(i) = Translation(orientation(0), orientation(1), orientation(2),
			position(0), position(1), position(2), sensorsValue.coords.row(i));
		temp1.row(i) = detectedCoord.row(i) - position;
		temp1.row(i)(2) = 0;
		// Khi cảm biến cảm nhận được vật thể nó sẽ trả về trạng thái kết nối.
		if (temp1.row(i).norm() <= 0.99) {
			status = 1;
		}
	}

	// Số lượng robot nhận thông tin
	soLuong = receiver->getQueueLength() - soLuong;
	std::cout << "So Luong:  " << receiver->getQueueLength() << std::endl;
	// Lấy dữ liệu data vào trong một lần và chạy. Khi chạy hết data sẽ bắt đầu tiếp nhận thông tin mới
	while (receiver->getQueueLength() > 0) {
		int sizeData = receiver->getDataSize();
		dataRecei = (infoRobot*)receiver->getData();
		infoRobot data_Recei = *dataRecei;
		if (data_Recei.state_nei == 0) {
			std::cout << "NameRei: " << data_Recei.robotName << std::endl;
			nei = 0;
		}
		receiver->nextPacket();
	}

	std::cout << "Status:  " << status << std::endl;
	std::cout << "Nei:  " << nei << std::endl;
	// Khi ở trạng thái có kết nối mới thực hiện các hành vi như bình thường
	if (status == 1) {
		if (nei == 1) {
			V = F1() * V_obs + F0() * V_m2g;


			double angle1 = atan2(0.0, 1.0);
			double angle2 = atan2(V_m2g(1), V_m2g(0));
			double desireOrientation = angle2 - angle1;
			double robotYaw = imu->getRollPitchYaw()[2];

			errorAngle = desireOrientation - robotYaw;

			if (abs(errorAngle) > M_PI)
			{
				if (errorAngle < 0)
				{
					errorAngle += 2 * M_PI;
				}
				else
				{
					errorAngle -= 2 * M_PI;
				}
			}



			errorQueue.push(errorAngle);

			if (errorQueue.size() > 20) errorQueue.pop();

			double errorSum = 0;
			for (int i = 0; i < errorQueue.size(); i++)
			{
				std::queue<double> temp = errorQueue;
				errorSum += temp.front();
				temp.pop();
			}


			//std::cout << "abs(obsOrientation - robotYaw):" << "\n";
			//std::cout << abs(obsOrientation - robotYaw) << "\n";

			//std::cout << "F1:" << "\n";
			//std::cout << F1() << "\n";

			//std::cout << "M_PI/6:" << "\n";
			//std::cout << M_PI / 6 << "\n";
			//std::cout << "-------------------------------------------------" << "\n";

			double v;
			if (F1() != 0)
			{
				if (abs(obsOrientation - robotYaw) < M_PI / 2.5)
				{
					v = 5 * V.norm();
				}
				else
				{
					v = 30 * V.norm();
				}
			}
			else
			{
				v = 100 * V.norm();
			}

			double omega = Kp * (errorAngle)+Ki * errorSum;

			double vr = (2 * v + omega * L) / 2 * R;
			double vl = (2 * v - omega * L) / 2 * R;


			/*std::cout << "robotyaw:" << "\n";
			std::cout << imu->getRollPitchYaw()[2] << "\n";
			std::cout << "-------------------------------------------------" << "\n";*/

			std::cout << "Velocity Robot: " << "\n";
			std::cout << "R: " << vr << "   L: " << vl << std::endl;

			velocityRobot = (vr + vl) / 2;


			if (v != 0)
			{
				rightMotor->setVelocity(vr);
				leftMotor->setVelocity(vl);
			}
			else
			{
				rightMotor->setVelocity(0.0);
				leftMotor->setVelocity(0.0);
			}
		}
		else {
			rightMotor->setVelocity(0.0);
			leftMotor->setVelocity(0.0);
		}
	}
	else {
		rightMotor->setVelocity(-1.0);
		leftMotor->setVelocity(1.0);
	}
}
