#include <iostream>
#include "NMEA.hpp"

static const char *sentence = "$GPGGA,053740.000,2503.6319,N,12136.0099,E,1,08,1.1,63.8,M,15.2,M,,0000*64\r\n";

int main(int _argc, char* _argv[])
{
	NMEA nmea;
	NMEA::NMEABase *b;
	NMEA::GGA *gga;
	const char *p;

	for (p = sentence; *p != 0; ++p)
	{
		switch (nmea.putChar(*p, &b))
		{
		case NMEA::nmeaComplete:
			gga = static_cast<NMEA::GGA*>(b);
			std::cout << (gga->lat / (60.0 * 10000.0)) << ", " <<
				(gga->lon / (60.0 * 10000.0)) << ", " <<
				gga->altMSL << " meters" << std::endl;
			break;
		case NMEA::nmeaCompleteInvalid:
			std::cout << "Checksum mismatch: " <<
				(int)b->messageChecksum << " != " <<
				(int)b->calculatedChecksum << std::endl;
			break;
		case NMEA::nmeaErrorReset:
			std::cout << "Parse error.\n";
			break;
		case NMEA::nmeaIncomplete:
			break;
		}
	}

	return 0;
}
