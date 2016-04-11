#include <unistd.h>
#include <cstdlib>
#include <csignal>
#include <cfloat>
#include <iostream>
#include <iomanip>
#include <string>
#include <AveragingBuffer.hpp>
#include <wiringPi.h>
#include "RpiDataSource.hpp"

using namespace std;

static int running = 1;

static void signalHandler(int _signum)
{
	switch (_signum)
	{
	case SIGINT:
	case SIGTERM:
		running = 0;
		break;
	}
}

#if 0
static void logCallback(const char *_fmt, ...)
{
	int len;
	va_list args;
	char *str;

	va_start(args, _fmt);
	len = vsnprintf(nullptr, 0, _fmt, args);

	if (len < 1)
		return;

	va_start(args, _fmt);
	str = new char[len + 1];
	vsnprintf(str, len + 1, _fmt, args);
	syslog(LOG_INFO, str);
	delete [] str;
}
#endif

int main(int _argc, char* _argv[])
{
	RpiDataSource rds;
	Data sample;
	RawData rawSample;
	DVector mMax = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
	DVector mMin = {DBL_MAX, DBL_MAX, DBL_MAX};
	DVector mBias, mScale, gBias;
	AveragingBuffer gx(400), gy(400), gz(400);
	string l;
	double avg;
	int i;

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	if (wiringPiSetup() != 0)
	{
		cout << "Failed to initialize wiringPi.\n";
		return -1;
	}

	cout << "Press enter, then wave autopilot unit in a figure-8...";
	getline(cin, l);

	if (!rds.start())
	{
		cerr << "Failed to start data source.\n";
		return -1;
	}

	for (i = 0; i < 400 && running != 0; ++i)
	{
		rds.rawSample(&rawSample);

		mMax.x = max(mMax.x, rawSample.m.x);
		mMax.y = max(mMax.y, rawSample.m.y);
		mMax.z = max(mMax.z, rawSample.m.z);

		mMin.x = min(mMin.x, rawSample.m.x);
		mMin.y = min(mMin.y, rawSample.m.y);
		mMin.z = min(mMin.z, rawSample.m.z);

		usleep(25000);
	}

	rds.stop();

	cout << endl;

	mBias.x = (mMax.x + mMin.x) / 2.0;
	mBias.y = (mMax.y + mMin.y) / 2.0;
	mBias.z = (mMax.z + mMin.z) / 2.0;

	cout << "Magnetometer Samples: " << i << endl <<
			"X bias : " << mBias.x << endl <<
			"Y bias : " << mBias.y << endl <<
			"Z bias : " << mBias.z << endl;

	mScale.x = (mMax.x - mMin.x) / 2.0;
	mScale.y = (mMax.y - mMin.y) / 2.0;
	mScale.z = (mMax.z - mMin.z) / 2.0;
	avg = (mScale.x + mScale.y + mScale.z) / 3.0;

	mScale.x = avg / mScale.x;
	mScale.y = avg / mScale.y;
	mScale.z = avg / mScale.z;

	cout << "X scale: " << mScale.x << endl <<
			"Y scale: " << mScale.y << endl <<
			"Z scale: " << mScale.z << endl << endl;

	cout << "Set the autopilot unit down on a level surface, then press enter...";
	getline(cin, l);

	if (!rds.start())
	{
		cerr << "Failed to start data source.\n";
		return -1;
	}

	for (i = 0; i < 400 && running != 0; ++i)
	{
		rds.rawSample(&rawSample);

		gBias.x = gx.pushSample(rawSample.g.x);
		gBias.y = gy.pushSample(rawSample.g.y);
		gBias.z = gz.pushSample(rawSample.g.z);

		usleep(25000);
	}

	rds.stop();

	cout << "Gyroscope Samples: " << i << endl <<
			"X bias : " << mBias.x << endl <<
			"Y bias : " << mBias.y << endl <<
			"Z bias : " << mBias.z << endl << endl;

	cout << "Press enter to run with bias / scale calculations...";

	if (!rds.start(gBias, mBias, mScale))
	{
		cerr << "Failed to start data source.\n";
		return -1;
	}

	while (running != 0)
	{
		rds.sample(&sample);

		if (sample.yaw < 0)
			sample.yaw += 360;

		cout << "\rYaw: " << setw(6) << setprecision(4) << sample.yaw << " " <<
				"Pitch: " << setw(6) << setprecision(4) << sample.pitch << " " <<
				"Roll: " << setw(6) << setprecision(4) << sample.roll;
		cout.flush();

		usleep(1000000);
	}

	rds.stop();

	return 0;
}
