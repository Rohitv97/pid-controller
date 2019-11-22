#include "aria.h"
#include<iostream>
#include<stdlib.h>
#include<math.h>
using namespace std;

int main(int argc, char **argv)
{
	//Initialise
	//create instances
	Aria::init();
	ArRobot robot;

	//parse command line arguments
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	//Connect to robot
	ArRobotConnector robotConnector(&argParser, &robot);

	if (robotConnector.connectRobot())
		std::cout << "Robot connected!" << std::endl;

	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	robot.setVel2(100, 100);

	double err_p, err_prev = 0.0, err_i = 0.0, err_d, tar = 450.0;
	//double kp = 0.732;
	//double ki = 0;
	//double kd = 0.1525;
	//

	double kp = 0.4;
	double ki = 0.00008;
	double kd = 0.1;

	int count = 0;
	
	while (true)
	{
		//--------Initialize values
		int base = 100, flag = 0;
		double distance = 0;
		

		//----------Get Sonar Readings----------
		
		ArSensorReading *sonarSensor[8];

		int sonarRange[8];
		for (int i = 0; i<8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}

		int min4 = sonarRange[4];
		int min5 = sonarRange[5];
		int min6 = sonarRange[6];
		int min7 = sonarRange[7];

		for (int i = 0; i<10; i++)
		{
			int val4 = robot.getSonarReading(4)->getRange();
			if (min4 > val4)
			{
				min4 = val4;
			}
		}

		for (int i = 0; i<10; i++)
		{
			int val5 = robot.getSonarReading(5)->getRange();
			if (min5 > val5)
			{
				min5 = val5;
			}
		}
		
		for( int i = 0; i<10; i++)
		{
			int val6 = robot.getSonarReading(6)->getRange();
			if(min6 > val6)
			{
				min6 = val6;
			}
		}
		
		if (min6 > 1000)
		{
			min6 = 1000;
		}

		for (int i = 0; i<10; i++)
		{
			int val7 = robot.getSonarReading(7)->getRange();
			if (min7 > val7)
			{
				min7 = val7;
			}
		}

		if(min7 > 1000)
		{
			min7 = 1000;
		}
		
		int x_dist = min6 * cos((50 * 180) / M_PI);

		if(min7<5000 && x_dist<5000)
		{
			distance = min(min7, x_dist);
		}
		if (abs(sonarRange[7] - x_dist) >= 1500)
		{
			distance = min(min7, x_dist);
		}
		if(min7<5000 && x_dist>=5000)
		{
			distance = min7;
		}
		if(min7>=5000 && x_dist<5000)
		{
			distance = x_dist;
		}

		double min45 = min(min4, min5);
		distance = min(distance, min45);

		if(min7>=5000 && x_dist>=5000)
		{
			printf("No suitable reading");
			flag = 1;
		}

		//----------------Calculate error------------
		else
		{
			err_p = tar - distance;
			err_i += err_p;
			err_d = err_p - err_prev;
			err_prev = err_p;
		}

		if(count==10)
		{
			cout << "Err_P: " << err_p << endl;
			cout << "Err_I: " << err_i << endl;
			cout << "Err_D: " << err_d << endl;
			count = 0;
		}
		count++;

		//-----------Calculate PID------------

		int output = (int)(kp * err_p + ki * err_i + kd * err_d);

		//---------Setting speed----------

		robot.setVel2(base - output, base);
		

		ArUtil::sleep(100);

	}

	//termination
	//stop the robot
	robot.lock();
	robot.stop();
	robot.unlock();

	//terminate all threads
	Aria::exit();


	return 0;
}
