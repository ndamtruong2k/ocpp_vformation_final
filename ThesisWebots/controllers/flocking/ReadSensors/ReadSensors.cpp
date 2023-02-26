#include "ReadSensors.h"


void ReadSensors::Init(webots::DistanceSensor* ds_[8], int timeStep)
{
	for (int i = 0; i < 8; i++)
	{
		ds[i] = ds_[i];
		ds[i]->enable(timeStep);
	}
	lineEquations = findLineEquations();

}

Eigen::Matrix<double, 1, 2> ReadSensors::findLineEquations()
{
	Eigen::Matrix<double, 1, 2> results;
	int lookup_table_rows = sizeof(lookup_table) / sizeof(lookup_table[0]);
	int lookup_table_cols = 2;
	for (int i = 0; i < lookup_table_rows - 1; i++)
	{
		double m = (lookup_table[i + 1][1] - lookup_table[i][1]) / (lookup_table[i + 1][0] - lookup_table[i][0]);
		double b = lookup_table[i + 1][1] - m * lookup_table[i + 1][0];

		results(i, 0) = m;
		results(i, 1) = b;
	}

	return results;
}


sensorValues ReadSensors::readDistSensors()
{
	sensorValues sensorValues;
	for (int i = 0; i < 8; i++)
	{
		double value = ds[i]->getValue();
		if (value >= lookup_table[1][1])
		{
			sensorValues.distances(i) = (value - lineEquations(0, 1)) / lineEquations(0, 0);
		}else
		{
			sensorValues.distances(i) = (value - lineEquations(1, 1)) / lineEquations(1, 0);
		}
		
		sensorValues.coords = ReadSensors::dist2Cord(sensorValues.distances);
	}
	/*std::cout << "-------------------------------------------------" << "\n";
	std::cout << sensorValues.distances << "\n";
	std::cout << "-------------------------------------------------" << "\n";*/
	
	return sensorValues;
}


Eigen::Matrix<double, 8, 3> ReadSensors::dist2Cord(Eigen::Vector<double, 8> sensorsValue)
{
	Eigen::Matrix<double, 8, 3> coord;
	for (int i = 0; i < 8; i++)
	{
		coord(i,0) = sensorsValue(i) * std::cos(sensorsOrientation[i]);
		coord(i,1) = sensorsValue(i) * std::sin(sensorsOrientation[i]);
		coord(i,2) = 0;
	}
	return coord;
}


