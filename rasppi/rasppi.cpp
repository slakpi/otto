#include <sys/types.h>
#include <unistd.h>
#include <cstdlib>
#include <csignal>
#include <cfloat>
#include <cstring>
#include <cstdarg>
#include <ctime>
#include <string>
#include <getopt.h>
#include <syslog.h>
#include <wiringPi.h>
#include <config.h>
#include <AveragingBuffer.hpp>
#include <FlightDirector.hpp>
#include <GISDatabase.hpp>
#include "RpiDataSource.hpp"
#include "RpiAutopilot.hpp"

using namespace std;

static const char recoveryDbOpt = 'd';
static const char helpOpt = 'h';
static const char *shortOpts = "d:h";
static const struct option longOpts[] = {
  { "recovery-database", required_argument, nullptr, recoveryDbOpt },
  { "help", no_argument, nullptr, helpOpt },
  { nullptr, 0, nullptr, 0 }
};

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

static void pulseLight(int _pulses)
{
	for ( ; _pulses > 0; --_pulses)
	{
		digitalWrite(1, HIGH);
		usleep(250000);

		digitalWrite(1, LOW);
		usleep(250000);
	}
}

static int calMag(RpiDataSource *_rds, DVector *_mBias, DVector *_mScale)
{
	RawData rawSample;
	DVector mMax = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
	DVector mMin = {DBL_MAX, DBL_MAX, DBL_MAX};
	double avg;
	int i;

	if (!_rds->start())
	{
		logCallback("OTTO: Failed to start Raspberry Pi Data Source.");
		return 0;
	}

	for (i = 0; i < 2400; ++i)
	{
		_rds->rawSample(&rawSample);

		mMax.x = max(mMax.x, rawSample.m.x);
		mMax.y = max(mMax.y, rawSample.m.y);
		mMax.z = max(mMax.z, rawSample.m.z);

		mMin.x = min(mMin.x, rawSample.m.x);
		mMin.y = min(mMin.y, rawSample.m.y);
		mMin.z = min(mMin.z, rawSample.m.z);

		usleep(25000);
	}

	_rds->stop();

	_mBias->x = (mMax.x + mMin.x) / 2.0;
	_mBias->y = (mMax.y + mMin.y) / 2.0;
	_mBias->z = (mMax.z + mMin.z) / 2.0;

	_mScale->x = (mMax.x - mMin.x) / 2.0;
	_mScale->y = (mMax.y - mMin.y) / 2.0;
	_mScale->z = (mMax.z - mMin.z) / 2.0;
	avg = (_mScale->x + _mScale->y + _mScale->z) / 3.0;

	_mScale->x = avg / _mScale->x;
	_mScale->y = avg / _mScale->y;
	_mScale->z = avg / _mScale->z;

	logCallback("OTTO: Magnetometer calibration complete.");

	return 1;
}

static int calGyro(RpiDataSource *_rds, DVector *_gBias)
{
	RawData rawSample;
	AveragingBuffer gx(400), gy(400), gz(400);
	int i;

	if (!_rds->start())
	{
		logCallback("OTTO: Failed to start Raspberry Pi Data Source.");
		return 0;
	}

	for (i = 0; i < 2400; ++i)
	{
		_rds->rawSample(&rawSample);

		_gBias->x = gx.pushSample(rawSample.g.x);
		_gBias->y = gy.pushSample(rawSample.g.y);
		_gBias->z = gz.pushSample(rawSample.g.z);

		usleep(25000);
	}

	_rds->stop();

	logCallback("OTTO: Gyroscope calibration complete.");

	return 1;
}

static void getRecoveryDbPath(string &_dbPath)
{
#ifdef WIN32
/* TODO: Get the path based on the ProgramData folder. */
#else
  _dbPath = INSTALL_PREFIX;

  if (!_dbPath.empty())
  {
    if (_dbPath.back() != '/')
      _dbPath.push_back('/');
  }

  _dbPath.append("share/otto/recovery.db");
#endif
}

int main(int _argc, char* _argv[])
{
  string dbPath;
  getRecoveryDbPath(dbPath);

  while (true)
  {
    int c = getopt_long(_argc, _argv, shortOpts, longOpts, nullptr);

    if (c == -1)
      break;

    switch (c)
    {
    case recoveryDbOpt:
      dbPath = optarg;
      break;
    case helpOpt:
      break;
    default:
      break;
    }
  }

	RpiDataSource *rds = new RpiDataSource();
	RpiAutopilot *ap = new RpiAutopilot();
	GISDatabase *db = new GISDatabase(dbPath.c_str());
	FlightDirector *fd = new FlightDirector(ap, rds, db, logCallback);
	DVector mBias, mScale, gBias;
	struct timespec start, end;
	u_int64_t diff;
	bool l = false;

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	if (wiringPiSetup() == -1)
	{
		logCallback("OTTO: Failed to start WiringPi library.");
		return -1;
	}

	pinMode(1, OUTPUT);
	digitalWrite(1, LOW);

// Pulse the light once to indicate magnetometer calibration
	pulseLight(1);
	usleep(5000000); //Give the user 5 seconds to prepare
	digitalWrite(1, HIGH);

	if (calMag(rds, &mBias, &mScale) != 1)
	{
		pulseLight(50); // Calibration failed
		return -1;
	}

	digitalWrite(1, LOW);

//	Pulse the light twice to indicate gyroscope calibration
	pulseLight(2);
	usleep(5000000); // Give the user 5 seconds to prepare
	digitalWrite(1, HIGH);

	if (calGyro(rds, &gBias) != 1)
	{
		pulseLight(50); // Calibration failed
		return -1;
	}

	digitalWrite(1, LOW);

//	Start up the Raspberry Pi Data Source with corrections
	if (!rds->start(gBias, mBias, mScale))
	{
		logCallback("OTTO: Failed to start Raspberry Pi Data Source.");
		return -1;
	}

	logCallback("OTTO: Beginning main loop...");

//	Main loop

	fd->enable();

	clock_gettime(CLOCK_MONOTONIC, &start);

	while (running != 0)
	{
		usleep(1000000);
		clock_gettime(CLOCK_MONOTONIC, &end);
		diff = 1000000000L * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;

		fd->refresh((double)diff / 1000.0);
		digitalWrite(1, (l = !l) ? HIGH : LOW);

		start = end;
	}

	delete fd; // FlightDirector deletes `ap', `rds', and `db'
	digitalWrite(1, LOW);
	logCallback("OTTO: Shutdown.");

	return 0;
}
