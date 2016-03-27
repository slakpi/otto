#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "LS20031.hpp"
#include "NMEA.hpp"

LS20031::LS20031()
:	fd(-1)
{

}

LS20031::~LS20031()
{
	uninit();
}

bool LS20031::init(const char *_dev)
{
	if (fd != -1)
		return false;

	fd = serialOpen(_dev, 57600);

	if (fd == -1)
		return false;

	return true;
}

bool LS20031::sample(GpsSample &_sample)
{
	return false;
}

void LS20031::uninit()
{
	if (fd == -1)
		return;

	close(fd);
	fd = -1;
}
