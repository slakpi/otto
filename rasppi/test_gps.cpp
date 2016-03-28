#include <iostream>
#include <csignal>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "NMEA.hpp"

static int run = 1;

static void signalHandler(int _signal)
{
	switch (_signal)
	{
	case SIGINT:
	case SIGTERM:
		run = 0;
		break;
	}
}

int main(int _argc, char* _argv[])
{
	char c;
	int fd = -1, g;
	NMEA nmea;
	NMEA::ParseStatus ret;
	NMEA::NMEABase *b;
	NMEA::GGA *gga;
	NMEA::VTG *vtg;

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	fd = serialOpen("/dev/ttyAMA0", 57600);

	if (fd == -1)
		return -1;

	serialPuts(fd, "$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");

	while (run)
	{
		g = serialDataAvail(fd);

		for ( ; g > 0; --g)
		{
			c = (char)serialGetchar(fd);
			ret = nmea.putChar(c, &b);

			switch (ret)
			{
			case NMEA::nmeaComplete:
				switch (b->getType())
				{
				case NMEA::nmeaGGA:
					gga = static_cast<NMEA::GGA*>(b);
					std::cout << (gga->lat / (60.0 * 10000.0)) << ", " <<
						(gga->lon / (60.0 * 10000.0)) << " " <<
						gga->altMSL << " meters" << std::endl;
					gga = NULL;
					break;
				case NMEA::nmeaVTG:
					vtg = static_cast<NMEA::VTG*>(b);
					std::cout << vtg->ktsGS << " kts, " <<
						vtg->kphGS << " kph, " <<
						vtg->trueGTK << " degress true, " <<
						vtg->mode << std::endl;
					vtg = NULL;
					break;
				}

				b->destroy();
				b = NULL;
				break;
			case NMEA::nmeaCompleteInvalid:
				std::cout << "Checksum mismatch " <<
					b->messageChecksum << " != " <<
					b->calculatedChecksum << std::endl;
				b->destroy();
				b = NULL;
				break;
			case NMEA::nmeaErrorReset:
				std::cout << "Parser error." << std::endl;
				break;
			case NMEA::nmeaIncomplete:
				break;
			}
		}

		sleep(1);
	}

	return 0;
}
