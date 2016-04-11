#include <sys/types.h>
#include <stdexcept>
#include <ctime>
#include <cstring>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "LSM6DS33.hpp"
#include "LIS3MDL.hpp"
#include "NMEA.hpp"
#include "Madgwick_AHRS.h"
#include "RpiDataSource.hpp"

#define DELAY 1250 /* 800 Hz */
#define RAD2DEGF(_r) ((float)((_r) * 180.0f / M_PI))
#define DEG2RADF(_d) ((float)((_d) * M_PI / 180.0f))

static const DVector zeroesVector = {0.0, 0.0, 0.0};
static const DVector onesVector = {1.0, 1.0, 1.0};

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

void* RpiDataSource::threadProc(void *_ptr)
{
	RpiDataSource *rds = static_cast<RpiDataSource*>(_ptr);
	LIS3MDL mag;
	LSM6DS33 imu;
	NMEA gpsParser;
	NMEA::ParseStatus parseRet;
	NMEA::NMEABase *b = NULL;
	NMEA::GGA *gga = NULL;
	NMEA::VTG *vtg = NULL;
	DVector m, a, g;
	float q[4], e[3];
	timespec tspec;
	int64_t t, r;
	int fd = -1, sd;
	bool gpsOk = false, magOk = false, imuOk = false;

/*	assume wiringPiSetup() has already been called. */

/*	setup the GPS. */

	fd = serialOpen("/dev/ttyAMA0", 57600);

	if (fd != -1)
	{
/* configure the GPS to only send GGA and VTG. */
		serialPuts(fd, "$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
		gpsOk = true;
	}

/*	initialize the IMU and magnetometer. */

	magOk = mag.init();
	imuOk = imu.init();

	m.x = m.y = m.z = 0.0;
	a.x = a.y = a.z = 0.0;
	g.x = g.y = g.z = 0.0;
	q[0] = q[1] = q[2] = q[3] = 0.0f;
	e[0] = e[1] = e[2] = 0.0f;

	clock_gettime(CLOCK_MONOTONIC, &tspec);
	r = tspec.tv_sec * 1000000LL;
	r += tspec.tv_nsec / 1000LL;

	while (true)
	{
		if (!__sync_bool_compare_and_swap(&rds->cancel, 0, 0))
			break;

		if (magOk)
		{
			mag.readMag(m);

			m.x = (m.x - rds->mBias.x) * rds->mScale.x;
			m.y = (m.y - rds->mBias.y) * rds->mScale.y;
			m.z = (m.z - rds->mBias.z) * rds->mScale.z;
		}

		if (imuOk)
		{
			imu.readGyro(g);
			imu.readAccel(a);

			g.x -= rds->gBias.x;
			g.y -= rds->gBias.y;
			g.z -= rds->gBias.z;
		}

		clock_gettime(CLOCK_MONOTONIC, &tspec);
		t = tspec.tv_sec * 1000000LL;
		t += tspec.tv_nsec / 1000LL;

		if (imuOk)
		{
			deltat = ((float)(t - r) / 1000000.0f);
			MadgwickAHRSupdate(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
			q[0] = q0;
			q[1] = q1;
			q[2] = q2;
			q[3] = q3;
			quaternionToYawPitchRoll(q, e);
			e[0] = (e[0] < 0 ? 360 + e[0] : e[0]);
		}

		r = t;

		if (gpsOk)
		{
			sd = serialDataAvail(fd);

			for ( ; sd > 0; --sd)
			{
				parseRet = gpsParser.putChar(static_cast<char>(serialGetchar(fd)), &b);

				switch (parseRet)
				{
				case NMEA::nmeaComplete:
					switch (b->getType())
					{
					case NMEA::nmeaGGA:
						if (gga != NULL)
							gga->destroy();

						gga = static_cast<NMEA::GGA*>(b);
						b = NULL;
						break;
					case NMEA::nmeaVTG:
						if (vtg != NULL)
							vtg->destroy();

						vtg = static_cast<NMEA::VTG*>(b);
						b = NULL;
						break;
					}

					break;
				case NMEA::nmeaCompleteInvalid:
					b->destroy();
					b = NULL;
					break;
				case NMEA::nmeaErrorReset:
					break;
				case NMEA::nmeaIncomplete:
					break;
				}
			}
		}

		pthread_mutex_lock(&rds->sampleLock);

/*	do not reset the `avail' flags in the current sample.  we are not
	guaranteed to get all parameters in each loop iteration.  so,
	leave the previous values.

	yaw data is only really going to be valid if both the IMU and the
	magnetometer are operational.
 */

			if (magOk)
				rds->curRawSample.m = m;

		 	if (imuOk)
		 	{
				if (magOk)
				{
					rds->curSample.avail |= DATA_YAW;
					rds->curSample.yaw = e[0];
				}

				rds->curRawSample.g = g;
				rds->curRawSample.a = a;
				rds->curSample.avail |= (DATA_PITCH | DATA_ROLL);
				rds->curSample.pitch = e[1];
				rds->curSample.roll = e[2];
			}

			if (gga != NULL)
			{
				rds->curSample.avail |= (DATA_POS | DATA_ALT);
				rds->curSample.pos.lat = gga->lat / (60.0 * 10000.0);
				rds->curSample.pos.lon = gga->lon / (60.0 * 10000.0);
				rds->curSample.alt = gga->altMSL * 3.28084; /* meters -> feet */
				gga->destroy();
				gga = NULL;
			}

			if (vtg != NULL)
			{
				rds->curSample.avail |= (DATA_HDG | DATA_GS);
				rds->curSample.hdg = vtg->magGTK;
				rds->curSample.gs = vtg->ktsGS;
				vtg->destroy();
				vtg = NULL;
			}

		pthread_mutex_unlock(&rds->sampleLock);

		usleep(DELAY);
	}

	pthread_exit(NULL);
}

RpiDataSource::RpiDataSource()
:	cancel(0),
	dataThread(0),
	sampleLock(PTHREAD_MUTEX_INITIALIZER)
{

}

RpiDataSource::~RpiDataSource()
{

}

bool RpiDataSource::start()
{
	return start(zeroesVector, zeroesVector, onesVector);
}

bool RpiDataSource::start(const DVector &_gBias, const DVector &_mBias, const DVector &_mScale)
{
	stop();

	memset(&curSample, 0, sizeof(curSample));
	memset(&curRawSample, 0, sizeof(curRawSample));
	gBias = _gBias;
	mBias = _mBias;
	mScale = _mScale;

	if (pthread_create(&dataThread, NULL, threadProc, this) != 0)
	{
		dataThread = 0;
		return false;
	}

	return true;
}

bool RpiDataSource::rawSample(RawData *_rawData) const
{
	RpiDataSource *rds = const_cast<RpiDataSource*>(this);

	pthread_mutex_lock(&rds->sampleLock);
	*_rawData = curRawSample;
	pthread_mutex_unlock(&rds->sampleLock);

	return true;
}

void RpiDataSource::stop()
{
	if (dataThread == 0)
		return;

	__sync_bool_compare_and_swap(&cancel, 0, 1);
	pthread_join(dataThread, NULL);
	dataThread = 0;
	cancel = 0;
}

bool RpiDataSource::sample(Data *_data) const
{
	RpiDataSource *rds = const_cast<RpiDataSource*>(this);

	pthread_mutex_lock(&rds->sampleLock);
	*_data = curSample;
	pthread_mutex_unlock(&rds->sampleLock);

	return true;
}
