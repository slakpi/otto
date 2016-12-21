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

#define APPLY_STATIC_HDG_OFFSET
#define APPLY_MAG_BIAS_CAL
//#define APPLY_MAG_SCALE_CAL
#define APPLY_GYRO_BIAS_CAL
#define DELAY 1250 /* 800 Hz */
#define RAD2DEGF(_r) ((float)((_r) * 180.0f / M_PI))
#define DEG2RADF(_d) ((float)((_d) * M_PI / 180.0f))

static const DVector zeroesVector = {0.0, 0.0, 0.0};

static const DVector onesVector = {1.0, 1.0, 1.0};

/*	fixed magnetic deviation corrections measured after applying dynamic
	hard- and soft-iron offset / scale corrections.
 */

static const float mDev[8] = {
	 -2, /*   0 */	  1, /*  45 */	 4, /*  90 */
 	  5, /* 135 */	  3, /* 180 */	 5, /* 225 */
	  4, /* 270 */	 -1, /* 315 */
};

static void quaternionToYawPitchRoll(float q[4], float e[3])
{
	float gx, gy, gz;

	gx = 2 * (q[1] * q[3] - q[0] * q[2]);
	gy = 2 * (q[0] * q[1] + q[2] * q[3]);
	gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	e[0] = RAD2DEGF(atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1));
	e[1] = RAD2DEGF(atan(gx / sqrt(gy * gy + gz * gz)));
	e[2] = RAD2DEGF(atan(gy / sqrt(gx * gx + gz * gz)));

/*	yaw is in the [-180, 180) range.  we need to correct this to
	[0, 360) as well as add an additional 90 degrees for the board
	orientation.
 */

	e[0] = e[0] < 0 ? e[0] + 450 : e[0] + 90;
	e[0] = e[0] >= 360 ? e[0] - 360 : e[0];

#ifdef APPLY_STATIC_HDG_OFFSET

/*	apply static magnetic deviation correction using simple linear
	interpolation.
 */

	if (e[0] < 45)			e[0] += (e[0]      ) * (mDev[1] - mDev[0]) / 45 + mDev[0];
	else if (e[0] < 90)		e[0] += (e[0] -  45) * (mDev[2] - mDev[1]) / 45 + mDev[1];
	else if (e[0] < 135)	e[0] += (e[0] -  90) * (mDev[3] - mDev[2]) / 45 + mDev[2];
	else if (e[0] < 180)	e[0] += (e[0] - 135) * (mDev[4] - mDev[3]) / 45 + mDev[3];
	else if (e[0] < 225)	e[0] += (e[0] - 180) * (mDev[5] - mDev[4]) / 45 + mDev[4];
	else if (e[0] < 270)	e[0] += (e[0] - 225) * (mDev[6] - mDev[5]) / 45 + mDev[5];
	else if (e[0] < 315)	e[0] += (e[0] - 270) * (mDev[7] - mDev[6]) / 45 + mDev[6];
	else					e[0] += (e[0] - 315) * (mDev[0] - mDev[7]) / 45 + mDev[7];

	e[0] = e[0] >= 360 ? e[0] - 360 : e[0];

#endif
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

/*	setup the Madgwick AHRS filter with a beta 20 times higher than
	the default.  this will let it settle on values faster.  presumably,
	there is some trade-off here.
 */

	beta = 2.0f;

/*	main loop. */

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

#ifdef APPLY_MAG_BIAS_CAL
			m.x -= rds->mBias.x;
			m.y -= rds->mBias.y;
			m.z -= rds->mBias.z;
#endif

#ifdef APPLY_MAG_SCALE_CAL
			m.x *= rds->mScale.x;
			m.y *= rds->mScale.y;
			m.z *= rds->mScale.z;
#endif
		}

		if (imuOk)
		{
			imu.readGyro(g);
			imu.readAccel(a);

#ifdef APPLY_GYRO_BIAS_CAL
			g.x -= rds->gBias.x;
			g.y -= rds->gBias.y;
			g.z -= rds->gBias.z;
#endif
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
	guaranteed to get all parameters during each loop iteration.  so,
	leave the previous values.
 */

			if (magOk)
				rds->curRawSample.m = m;

		 	if (imuOk)
		 	{
				if (magOk)
				{

/*	yaw data is only really going to be valid if both the IMU and the
	magnetometer are operational.
 */

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
	stop();
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
