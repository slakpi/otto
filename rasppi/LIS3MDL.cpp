#include <unistd.h>
#include <wiringPiI2C.h>
#include "LIS3MDL.hpp"

#define LIS3MDL_SA1_HIGH_ADDRESS	0x1e
#define LIS3MDL_SA1_LOW_ADDRESS		0x1c
#define LIS3MDL_WHO_ID				0x3d
#define INVALID_FD					-1

/*	CTRL_1_DEFAULT

	1       00 110 0        0
	-       -- --- -        -
	TEMP_EN OM DO  FAST_ODR ST

	TEMP_EN		b1		= Temperature sensor enabled
	OM			b00		= X & Y in low power mode
	DO			b110	= 40 Hz
	FAST_ODR	b0		= Fast ODR disabled
	ST			b0		= Self-test disabled
 */

#define CTRL_1_DEFAULT				0x98

/*	CTRL_2_DEFAULT

	0 01 0 0      0        0 0
	- -- - -      -        - -
	0 FS 0 REBOOT SOFT_RST 0 0

	FS			b01		= +/- 8 gauss
 */

#define CTRL_2_DEFAULT				0x20
#define MIN_MAG						-8
#define MAX_MAG						8

/*	CTRL_3_DEFAULT

	0 0 0  0 0 0   00
	- - -  - - -   --
	0 0 LP 0 0 SIM MD

	LP			b0		= Low power mode disabled
	SIM			b0		= SPI serial interface disabled
	MD			b00		= Continuous conversion mode
 */

#define CTRL_3_DEFAULT				0x00

/*	CTRL_4_DEFAULT

	0 0 0 0 00  0   0
	- - - - --  -   -
	0 0 0 0 OMZ BLE 0

	OMZ			b00		= Low power mode for Z-axis
	BLE			b0		= LSb at lower address
 */

#define CTRL_4_DEFAULT				0x00

static double makeMag(int16_t _v)
{
	return (double)((_v + 32768) * ((MAX_MAG - MIN_MAG) / 65535.0) + MIN_MAG);
}

LIS3MDL::LIS3MDL()
:	fd(INVALID_FD)
{

}

LIS3MDL::~LIS3MDL()
{
	uninit();
}

bool LIS3MDL::init(Sa1State _sa1 /* = sa1_auto */)
{
	if (fd != INVALID_FD)
		return true;
	if (_sa1 != sa1_low && init2(LIS3MDL_SA1_HIGH_ADDRESS))
		return true;
	if (_sa1 != sa1_high && init2(LIS3MDL_SA1_LOW_ADDRESS))
		return true;

	return false;
}

void LIS3MDL::uninit()
{
	if (fd == INVALID_FD)
		return;

	close(fd);
	fd = INVALID_FD;
}

void LIS3MDL::readMag(Vector<double> &_m) const
{
	u_int8_t lx, hx, ly, hy, lz, hz;

	_m.x = _m.y = _m.z = 0;

	if (fd == INVALID_FD)
		return;

	lx = wiringPiI2CReadReg8(fd, OUT_X_L);
	hx = wiringPiI2CReadReg8(fd, OUT_X_H);
	ly = wiringPiI2CReadReg8(fd, OUT_Y_L);
	hy = wiringPiI2CReadReg8(fd, OUT_Y_H);
	lz = wiringPiI2CReadReg8(fd, OUT_Z_L);
	hz = wiringPiI2CReadReg8(fd, OUT_Z_H);

	_m.x = makeMag((hx << 8) | lx);
	_m.y = makeMag((hy << 8) | ly);
	_m.z = makeMag((hz << 8) | lz);
}

bool LIS3MDL::init2(u_int8_t _addr)
{
	u_int16_t r;

	fd = wiringPiI2CSetup(_addr);

	if (fd != INVALID_FD)
	{
		r = wiringPiI2CReadReg8(fd, WHO_AM_I);

		if (r == LIS3MDL_WHO_ID)
		{
			r = wiringPiI2CWriteReg8(fd, CTRL_REG1, CTRL_1_DEFAULT);
			r = wiringPiI2CWriteReg8(fd, CTRL_REG2, CTRL_2_DEFAULT);
			r = wiringPiI2CWriteReg8(fd, CTRL_REG3, CTRL_3_DEFAULT);
			r = wiringPiI2CWriteReg8(fd, CTRL_REG4, CTRL_4_DEFAULT);
			return true;
		}

		close(fd);
		fd = INVALID_FD;
	}

	return false;
}
