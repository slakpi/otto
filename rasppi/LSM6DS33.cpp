#include <unistd.h>
#include <wiringPiI2C.h>
#include "LSM6DS33.hpp"

#define DS33_SA0_HIGH_ADDRESS	0x6b
#define DS33_SA0_LOW_ADDRESS	0x6a
#define DS33_WHO_ID				0x69

/*	ACCEL_MODE_DEFAULT

	0110  10  00
    ----  --  --
    ODR   FS  BW

	ODR	b0110	= 416 Hz
	FS	b10 	= +/- 4g
	BW	b00		= 400 Hz AA filter
 */

#define ACCEL_MODE_DEFAULT		0x68
#define MIN_ACCEL				-4
#define MAX_ACCEL				 4

/*	GYRO_MODE_DEFAULT

	0011  00  0       0
    ----  --  -       -
    ODR   FS  125 dps X

	ODR	b0011	= 52 Hz (low power)
	FS	b00 	= 245 dps (+/- 122.5 dps)
 */

#define GYRO_MODE_DEFAULT		0x30
#define MIN_GYRO				-122.5
#define MAX_GYRO				 122.5

/*	COMMON_MODE_DEFAULT

    0    0   0         0     0   0      0   0
    -    -   -         -     -   -      -   -
	BOOT BDU H_LACTIVE PP_OD SIM IF_INC BLE SW_RESET
 */

#define COMMON_MODE_DEFAULT		0x00

static double makeAccel(int16_t _v)
{
	return static_cast<double>(((_v + 32768) * ((MAX_ACCEL - MIN_ACCEL) / 65535.0) + MIN_ACCEL));
}

static double makeGyro(int16_t _v)
{
	return static_cast<double>(((_v + 32768) * ((MAX_GYRO - MIN_GYRO) / 65535.0) + MIN_GYRO));
}

LSM6DS33::LSM6DS33()
:	fd(-1)
{

}

LSM6DS33::~LSM6DS33()
{
	uninit();
}

bool LSM6DS33::init(Sa0State _sa0 /* = sa0_auto */)
{
	if (fd != -1)
		return true;
	if (_sa0 != sa0_low && init2(DS33_SA0_HIGH_ADDRESS))
		return true;
	if (_sa0 != sa0_high && init2(DS33_SA0_LOW_ADDRESS))
		return true;

	return false;
}

void LSM6DS33::uninit()
{
	if (fd == -1)
		return;

	close(fd);
	fd = -1;
}

void LSM6DS33::readAccel(DVector &_a) const
{
	u_int8_t lx, hx, ly, hy, lz, hz;

	_a.x = _a.y = _a.z = 0;

	if (fd == -1)
		return;

	lx = wiringPiI2CReadReg8(fd, OUTX_L_XL);
	hx = wiringPiI2CReadReg8(fd, OUTX_H_XL);
	ly = wiringPiI2CReadReg8(fd, OUTY_L_XL);
	hy = wiringPiI2CReadReg8(fd, OUTY_H_XL);
	lz = wiringPiI2CReadReg8(fd, OUTZ_L_XL);
	hz = wiringPiI2CReadReg8(fd, OUTZ_H_XL);

	_a.x = makeAccel((hx << 8) | lx);
	_a.y = makeAccel((hy << 8) | ly);
	_a.z = makeAccel((hz << 8) | lz);
}

void LSM6DS33::readGyro(DVector &_g) const
{
	u_int8_t lx, hx, ly, hy, lz, hz;

	_g.x = _g.y = _g.z = 0;

	if (fd == -1)
		return;

	lx = wiringPiI2CReadReg8(fd, OUTX_L_G);
	hx = wiringPiI2CReadReg8(fd, OUTX_H_G);
	ly = wiringPiI2CReadReg8(fd, OUTY_L_G);
	hy = wiringPiI2CReadReg8(fd, OUTY_H_G);
	lz = wiringPiI2CReadReg8(fd, OUTZ_L_G);
	hz = wiringPiI2CReadReg8(fd, OUTZ_H_G);

	_g.x = makeGyro((hx << 8) | lx);
	_g.y = makeGyro((hy << 8) | ly);
	_g.z = makeGyro((hz << 8) | lz);
}

bool LSM6DS33::init2(u_int8_t _addr)
{
	int r;

	fd = wiringPiI2CSetup(_addr);

	if (fd == -1)
		return false;

	r = wiringPiI2CReadReg8(fd, WHO_AM_I);

	if (r == DS33_WHO_ID)
	{
		r = wiringPiI2CWriteReg8(fd, CTRL1_XL, ACCEL_MODE_DEFAULT);
		r = wiringPiI2CWriteReg8(fd, CTRL2_G, GYRO_MODE_DEFAULT);
		r = wiringPiI2CWriteReg8(fd, CTRL3_C, COMMON_MODE_DEFAULT);
		return true;
	}

	close(fd);
	fd = -1;

	return false;
}
