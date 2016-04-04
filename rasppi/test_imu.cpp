#include <sys/types.h>
#include <iostream>
#include <csignal>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include <wiringPi.h>
#include <AveragingBuffer.hpp>
#include "LSM6DS33.hpp"
#include "LIS3MDL.hpp"
#include "HD44780.hpp"
#include "Mahony_AHRS.h"

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

#define RAD2DEGF(_r) ((float)((_r) * 180.0f / M_PI))

void quaternionToEuler(float q[4], float e[3])
{
   float sqw = q[0] * q[0];
   float sqx = q[1] * q[1];
   float sqy = q[2] * q[2];
   float sqz = q[3] * q[3];
   e[0] = atan2f(2.f * (q[1]*q[2] + q[3]*q[0]), sqx - sqy - sqz + sqw);
   e[1] = asinf(-2.f * (q[1]*q[3] - q[2]*q[0]));
   e[2] = atan2f(2.f * (q[2]*q[3] + q[1]*q[0]), -sqx - sqy + sqz + sqw);
}

int main(int _argc, char* _argv[])
{
	LIS3MDL imuMag;
	LSM6DS33 imuGyroAccel;
	HD44780 lcd;
	Vector<double> m, a, g;
	int64_t t, r;
	timespec spec;
	char buf[17];
	float q[4], e[3];

	if (wiringPiSetup() == -1)
	{
		cerr << "Failed to initialize wiringPi.\n";
		return -1;
	}

	if (!lcd.init())
	{
		cerr << "Failed to initialize LCD driver.\n";
		return -1;
	}

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

	cout << "t,Mx,My,Mz,Ax,Ay,Az,Gx,Gy,Gz,P,R,Y\n";

	while (run)
	{
		clock_gettime(CLOCK_MONOTONIC, &spec);
		t = spec.tv_sec * 1000000LL;
		t += spec.tv_nsec / 1000LL;
		t -= r;

		imuMag.readMag(m);
		imuGyroAccel.readGyro(g);
		imuGyroAccel.readAccel(a);

		MahonyAHRSupdate(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
		q[0] = q0;
		q[1] = q1;
		q[2] = q2;
		q[3] = q3;
		quaternionToEuler(q, e);

		cout << m.x << "," << m.y << "," << m.z << "," << a.x << "," <<
			a.y << "," << a.z << "," << g.x << "," << g.y << "," <<
			g.z << "," << RAD2DEGF(e[0]) << "," << RAD2DEGF(e[1]) << "," <<
			RAD2DEGF(e[2]) << endl;

		snprintf(buf, 17, "%6.1f %6.1f", RAD2DEGF(e[0]), RAD2DEGF(e[1]));
		lcd.setCursorPos(0, 0);
		lcd.writeString(buf);

		snprintf(buf, 17, "%6.1f", RAD2DEGF(e[2]));
		lcd.setCursorPos(1, 0);
		lcd.writeString(buf);

		usleep(DELAY);
	}

	return 0;
}
