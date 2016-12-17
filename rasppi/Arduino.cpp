#include <unistd.h>
#include <wiringPiI2C.h>
#include <Utilities.hpp>
#include "Arduino.hpp"

#define ARDUINO_ADDRESS			0x08

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

	wiringPiI2CWriteReg8(fd, LED_REG, _on ? 1 : 0);
}

float Arduino::getServoPos() const
{
	int Pdeg;

	if (fd == -1)
		return 0.0f;

/*	See setServoPos() for the scaling function.  This is just the
	reverse of that function.
 */

	Pdeg = wiringPiI2CReadReg8(fd, SERVO_REG);
	Pdeg = clamp(Pdeg, 60, 120);
	return (static_cast<float>(Pdeg - 60) * (2.0f / 60.0f) - 1.0f);
}

void Arduino::setServoPos(float _Prel)
{
	u_int8_t Pdeg;

	if (fd == -1)
		return;

/*	The servo operates in the range [0, 180] degrees where 90 degrees
	is center.  We're going to limit the range to [60, 120] degrees
	or center -/+ 30 degrees.

	Clamp the relative position value to the range [-1, 1], then scale
	the value to the range [-30, 30] and shift it up to the range
	[60, 120].

                         120 - 60
	Pdeg = (Prel - -1) * -------- - 30 + 90
                          1 - -1
 */

	_Prel = clamp(_Prel, -1.0f, 1.0f);
	Pdeg = static_cast<u_int8_t>((_Prel + 1.0f) * 30.0f + 60.0f);
	wiringPiI2CWriteReg8(fd, SERVO_REG, Pdeg);
}
