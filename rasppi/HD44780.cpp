#include <unistd.h>
#include <wiringPi.h>
#include <Utilities.hpp>
#include "HD44780.hpp"

//									  Raspberry Pi 3		HD44780
//					wiringPi		BCM			Physical	Package
//					---------------------------------------------------
//		Vss (0V)											1
//		Vdd (+5V)											2
//		Vo  (LCD)											3
#define PIN_REG     7			//	4			7			4
#define PIN_RW      0			//	17			11			5
#define PIN_E       24			//	19			35			6
#define PIN_DB0     2			//	27			13			7
#define PIN_DB1     3			//	22			15			8
#define PIN_DB2     4			//	23			16			9
#define PIN_DB3     5			//	24			18			10
#define PIN_DB4     6			//	25			22			11
#define PIN_DB5     21			//	5			29			12
#define PIN_DB6     22			//	6			31			13
#define PIN_DB7     23			//	13			33			14
//		Backlight 1											15
//		Backlight 2											16

static const int pins[] = {
    PIN_DB0,    PIN_DB1,    PIN_DB2,    PIN_DB3,
    PIN_DB4,    PIN_DB5,    PIN_DB6,    PIN_DB7
};

static void writeCommand(unsigned char _c)
{
    digitalWrite(PIN_E, 1);
    digitalWrite(PIN_REG, 0);
    digitalWrite(PIN_RW, 0);

    for (int i = 0; i < 8; ++i)
        digitalWrite(pins[i], _c & 0x1), _c >>= 1;

    usleep(3000);

    digitalWrite(PIN_E, 0);
}

static void writeData(unsigned char _c)
{
    digitalWrite(PIN_E, 1);
    digitalWrite(PIN_REG, 1);
    digitalWrite(PIN_RW, 0);

    for (int i = 0; i < 8; ++i)
        digitalWrite(pins[i], _c & 0x1), _c >>= 1;

    usleep(3000);

    digitalWrite(PIN_E, 0);
}

HD44780::HD44780()
:	initialized(false)
{

}

HD44780::~HD44780()
{

}

bool HD44780::init()
{
	if (initialized)
		return true;

/*	assuming wiringPi has already been initialized using wPi pins. */

    pinMode(PIN_REG, OUTPUT);
    pinMode(PIN_RW, OUTPUT);
    pinMode(PIN_DB0, OUTPUT);
    pinMode(PIN_DB1, OUTPUT);
    pinMode(PIN_DB2, OUTPUT);
    pinMode(PIN_DB3, OUTPUT);
    pinMode(PIN_DB4, OUTPUT);
    pinMode(PIN_DB5, OUTPUT);
    pinMode(PIN_DB6, OUTPUT);
    pinMode(PIN_DB7, OUTPUT);
    pinMode(PIN_E, OUTPUT);

    writeCommand(0x0c); // display on with blinking cursor
    writeCommand(0x3c); // 8-bit, two lines
    writeCommand(0x10); // move cursor rather than shift display
    writeCommand(0x06); // increment cursor, no display shift
    writeCommand(0x01); // clear the display
    writeCommand(0x02); // move the cursor home

	initialized = true;

	return true;
}

void HD44780::clear()
{
	if (!initialized)
		return;

	writeCommand(0x01); // clear the display
	writeCommand(0x02); // move the cursor home
}

void HD44780::setCursorPos(int _line, int _column)
{
	unsigned char cmd = 0x80;

	if (!initialized)
		return;

	_line = clamp(_line, 0, 1);
	_column = clamp(_column, 0, 15);

	if (_line == 1)
		cmd += 0x40;

	cmd += (unsigned char)(_column & 0x0f);

	writeCommand(cmd);
}

void HD44780::writeChar(char _c)
{
	if (!initialized)
		return;

	writeData(clamp(_c, (char)0x20, (char)0x7f));
}

void HD44780::writeString(const char *_str)
{
	if (!initialized)
		return;

	for (const char *p = _str; p != NULL && *p != 0; ++p)
		writeData(clamp(*p, (char)0x20, (char)0x7f));
}
