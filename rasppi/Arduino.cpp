#include <unistd.h>
#include <wiringPiI2C.h>
#include "Arduino.hpp"

#define ARDUINO_ADDRESS			0x08

#define TEST_REG				0xc0

Arduino::Arduino()
:	fd(-1)
{

}

Arduino::~Arduino()
{
	uninit();
}

bool Arduino::init()
{
	fd = wiringPiI2CSetup(ARDUINO_ADDRESS);

	if (fd == -1)
		return false;

	return true;
}

void Arduino::uninit()
{
	if (fd == -1)
		return;

	close(fd);
	fd = -1;
}

void Arduino::setLight(bool _on)
{
	if (fd == -1)
		return;

	wiringPiI2CWriteReg8(fd, TEST_REG, _on ? 1 : 0);
}
