#include <sys/types.h>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include <wiringPi.h>
#include <AveragingBuffer.hpp>
#include "LSM6DS33.hpp"
#include "LIS3MDL.hpp"
#include "HD44780.hpp"
#include "Madgwick_AHRS.h"

#define NO_LCD_OUTPUT
#define DELAY			2403
//#define NO_BIAS_REMOVAL
#define BIAS_SAMPLES 	512
#define BIAS_WAIT		10 * 1000000

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
#define DEG2RADF(_d) ((float)((_d) * M_PI / 180.0f))

static void quaternionToYawPitchRoll(float q[4], float e[3])
{
	float gx, gy, gz;

	gx = 2 * (q[1] * q[3] - q[0] * q[2]);
	gy = 2 * (q[0] * q[1] + q[2] * q[3]);
	gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	e[0] = RAD2DEGF(atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1));
	e[1] = RAD2DEGF(atan(gx / sqrt(gy * gy + gz * gz)));
	e[2] = RAD2DEGF(atan(gy / sqrt(gx * gx + gz * gz)));
}

int main(int _argc, char* _argv[])
{
	LIS3MDL imuMag;
	LSM6DS33 imuGyroAccel;
#ifndef NO_LCD_OUTPUT
	HD44780 lcd;
	char buf[17];
#endif
	DVector m, a, g, gb;
	AveragingBuffer gbx(BIAS_SAMPLES), gby(BIAS_SAMPLES), gbz(BIAS_SAMPLES);
	int64_t t, t1, r;
	timespec spec;
	float q[4], e[3];

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	if (wiringPiSetup() == -1)
	{
		cerr << "Failed to initialize wiringPi.\n";
		return -1;
	}

#ifndef NO_LCD_OUTPUT
	if (!lcd.init())
	{
		cerr << "Failed to initialize LCD driver.\n";
		return -1;
	}
#endif

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

#ifndef NO_LCD_OUTPUT
#ifndef NO_BIAS_REMOVAL
	lcd.clear();

	snprintf(buf, 17, "  Calibrating   ");
	lcd.setCursorPos(0, 0);
	lcd.writeString(buf);

	snprintf(buf, 17, "  KEEP  LEVEL   ");
	lcd.setCursorPos(1, 0);
	lcd.writeString(buf);
#endif
#endif

	clock_gettime(CLOCK_MONOTONIC, &spec);
	r = spec.tv_sec * 1000000LL;
	r += spec.tv_nsec / 1000LL;
	t1 = r;

	cout << "t,Mx,My,Mz,Ax,Ay,Az,Gx,Gy,Gz,q0,q1,q2,q3,Y,P,R\n";

	while (run)
	{
		clock_gettime(CLOCK_MONOTONIC, &spec);
		t = spec.tv_sec * 1000000LL;
		t += spec.tv_nsec / 1000LL;
		t -= r;

		imuMag.readMag(m);
		imuGyroAccel.readGyro(g);
		imuGyroAccel.readAccel(a);

#ifndef NO_BIAS_REMOVAL
		g.x -= (gb.x = gbx.pushSample(g.x));
		g.y -= (gb.y = gby.pushSample(g.y));
		g.z -= (gb.z = gbz.pushSample(g.z));
#endif

		g.x = DEG2RADF(g.x);
		g.y = DEG2RADF(g.y);
		g.z = DEG2RADF(g.z);

		deltat = ((float)(t - t1) / 1000000.0f);
		MadgwickAHRSupdate(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
		q[0] = q0;
		q[1] = q1;
		q[2] = q2;
		q[3] = q3;

#ifndef NO_BIAS_REMOVAL
		if (t > BIAS_WAIT)
		{
#endif
			quaternionToYawPitchRoll(q, e);

			cout << t << "," <<
				m.x << "," << m.y << "," << m.z << "," <<
				a.x << "," << a.y << "," << a.z << "," <<
				g.x << "," << g.y << "," << g.z << "," <<
				q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "," <<
				e[0] << "," << e[1] << "," << e[2] << endl;

#ifndef NO_LCD_OUTPUT
			snprintf(buf, 17, "M%6d Y%6.1f", (int)hdg, e[0]);
			lcd.setCursorPos(0, 0);
			lcd.writeString(buf);

			snprintf(buf, 17, "P%6.1f R%6.1f", e[1], e[2]);
			lcd.setCursorPos(1, 0);
			lcd.writeString(buf);
#endif
#ifndef NO_BIAS_WAIT
		}
#endif

		t1 = t;
		usleep(DELAY);
	}

	return 0;
}
