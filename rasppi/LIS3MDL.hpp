#ifndef LIS3MDL_HPP
#define LIS3MDL_HPP

#include <sys/types.h>
#include "Vector.hpp"

class LIS3MDL
{
public:
	enum Sa1State
	{
		sa1_low,
		sa1_high,
		sa1_auto
	};

	enum RegAddr
	{
		WHO_AM_I    = 0x0F,

		CTRL_REG1   = 0x20,
		CTRL_REG2   = 0x21,
		CTRL_REG3   = 0x22,
		CTRL_REG4   = 0x23,
 		CTRL_REG5   = 0x24,

		STATUS_REG  = 0x27,
		OUT_X_L     = 0x28,
		OUT_X_H     = 0x29,
		OUT_Y_L     = 0x2A,
		OUT_Y_H     = 0x2B,
		OUT_Z_L     = 0x2C,
		OUT_Z_H     = 0x2D,
		TEMP_OUT_L  = 0x2E,
		TEMP_OUT_H  = 0x2F,
		INT_CFG     = 0x30,
		INT_SRC     = 0x31,
		INT_THS_L   = 0x32,
		INT_THS_H   = 0x33
	};

public:
	LIS3MDL();

public:
	~LIS3MDL();

public:
    bool init(Sa1State _sa1 = sa1_auto);

	void uninit();

	void readMag(Vector<double> &_m) const;

private:
	bool init2(u_int8_t _addr);

private:
	int fd;
};

#endif
