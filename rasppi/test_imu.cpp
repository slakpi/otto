#include <iostream>
#include <csignal>
#include <unistd.h>
#include "LIS3MDL.hpp"

static int run = 1;

static void signalHandler(int _signal)
{
	switch (_signal)
	{
	case SIGINT:
	case SIGTERM:
		run = 0;
		break;
	}
}

int main(int _argc, char* _argv[])
{
	LIS3MDL imu;
	Vector<double> m;
	double hdg;

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	if (!imu.init())
	{
		std::cout << "Failed to initialize IMU.\n";
		return -1;
	}

	while (run)
	{
		imu.readMag(m, hdg);
		std::cout << hdg << std::endl;
		sleep(1);
	}

	return 0;
}
