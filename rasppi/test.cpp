#include <iostream>
#include <csignal>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "NMEA.hpp"

int main(int _argc, char* _argv[])
{
	char c;
	int fd = -1, g;
	NMEA nmea;
	NMEA::ParseStatus ret;
	NMEA::NMEABase *b;
	NMEA::GGA *gga;

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	fd = serialOpen("/dev/ttyAMA0", 57600);

	if (fd == -1)
	{
		switch (nmea.putChar(*p, &b))
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
