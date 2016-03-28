#include <sys/types.h>
#include <iostream>
#include <csignal>
#include <ctime>
#include <unistd.h>
#include <AveragingBuffer.hpp>
#include "LSM6DS33.hpp"
#include "LIS3MDL.hpp"

#define DELAY 20000

using namespace std;

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
	LIS3MDL imuMag;
	LSM6DS33 imuGyroAccel;
	Vector<double> m, a, aa, g, ga;
	AveragingBuffer gx(3), gy(3), gz(3);
	AveragingBuffer ax(3), ay(3), az(3);
	double hdg;
	int64_t t, r;
	timespec spec;

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	if (!imuMag.init())
	{
		cerr << "Failed to initialize IMU magnetometer.\n";
		return -1;
	}

	if (!imuGyroAccel.init())
	{
		cerr << "Failed to initialize IMU gyroscope/accelerometer.\n";
		return -1;
	}

	clock_gettime(CLOCK_MONOTONIC, &spec);
	r = spec.tv_sec * 1000000LL;
	r += spec.tv_nsec / 1000LL;

	cout << "t,Mx,My,Mz,Hdg,Ax,Ay,Az,Axa,Aya,Aza,Gx,Gy,Gz,Gxa,Gya,Gyz\n";

	while (run)
	{
		clock_gettime(CLOCK_MONOTONIC, &spec);
		t = spec.tv_sec * 1000000LL;
		t += spec.tv_nsec / 1000LL;
		t -= r;

		imuMag.readMag(m, hdg);
		imuGyroAccel.readGyro(g);
		imuGyroAccel.readAccel(a);

		aa.x = a.x - ax.pushSample(a.x);
		aa.y = a.y - ay.pushSample(a.y);
		aa.z = a.z - az.pushSample(a.z);

		ga.x = g.x - gx.pushSample(g.x);
		ga.y = g.y - gy.pushSample(g.y);
		ga.z = g.z - gz.pushSample(g.z);

		cout << t << "," << m.x << "," << m.y << "," << m.z << "," <<
			hdg << "," << a.x << "," << a.y << "," << a.z << "," <<
			aa.x << "," << aa.y << "," << aa.z << "," << g.x << "," <<
			g.y << "," << g.z << "," << ga.x << "," << ga.y << "," <<
			ga.z << endl;

		usleep(DELAY);
	}

	return 0;
}
