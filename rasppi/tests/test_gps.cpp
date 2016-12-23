#include <sys/types.h>
#include <iostream>
#include <csignal>
#include <ctime>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <NMEA.hpp>
#include <HD44780.hpp>

#define DELAY 100000

using namespace std;

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
	int fd = -1, g;
	NMEA nmea;
	NMEA::ParseStatus ret;
	NMEA::NMEABase *b = NULL;
	NMEA::GGA *gga = NULL;
	NMEA::VTG *vtg = NULL;
	HD44780 lcd;
	timespec spec;
	int64_t t, r;
	int d;
	double m;
	char buf[17];

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	if (wiringPiSetup() == -1)
	{
		cerr << "Failed to setup wiringPi.\n";
		return -1;
	}

	if (!lcd.init())
	{
		cerr << "Failed to setup LCD driver.\n";
		return -1;
	}

	fd = serialOpen("/dev/ttyAMA0", 57600);

	if (fd == -1)
	{
		cerr << "Failed to connect to GPS.\n";
		return -1;
	}

	serialPuts(fd, "$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");

	clock_gettime(CLOCK_MONOTONIC, &spec);
	r = spec.tv_sec * 1000000LL;
	r += spec.tv_nsec / 1000LL;

	cout << "t,UTC,Lat,Lon,Alt,MagGTK,KtsGS\n";

	while (run)
	{
		g = serialDataAvail(fd);

		for ( ; g > 0 && run; --g)
		{
			ret = nmea.putChar(static_cast<char>(serialGetchar(fd)), &b);

			switch (ret)
			{
			case NMEA::nmeaComplete:
				switch (b->getType())
				{
				case NMEA::nmeaGGA:
					if (gga != NULL) gga->destroy();
					gga = static_cast<NMEA::GGA*>(b);
					b = NULL;
					break;
				case NMEA::nmeaVTG:
					if (vtg != NULL) vtg->destroy();
					vtg = static_cast<NMEA::VTG*>(b);
					b = NULL;
					break;
				}

				break;
			case NMEA::nmeaCompleteInvalid:
				b->destroy();
				b = NULL;
				break;
			case NMEA::nmeaErrorReset:
				break;
			case NMEA::nmeaIncomplete:
				break;
			}

			if (gga != NULL && vtg != NULL)
			{
				clock_gettime(CLOCK_MONOTONIC, &spec);
				t = spec.tv_sec * 1000000LL;
				t += spec.tv_nsec / 1000LL;
				t -= r;

				cout << t << "," << gga->utc << "," <<
					(gga->lat / (60.0 * 10000.0)) << "," <<
					(gga->lon / (60.0 * 10000.0)) << "," <<
					gga->altMSL << "," << vtg->magGTK << "," <<
					vtg->ktsGS << endl;

				d = (int)(gga->lat / (60.0 * 10000.0));
				if (d < 0)
					d = -d;

				m = ((gga->lat % 600000) / 10000.0);
				if (m < 0)
					m = -m;

				snprintf(buf, 17, "%c%3d %.1f GS%3d", gga->lat < 0 ? 'S' : 'N', d, m, (int)vtg->ktsGS);
				lcd.setCursorPos(0, 0);
				lcd.writeString(buf);

				d = (int)(gga->lon / (60.0 * 10000.0));
				if (d < 0)
					d = -d;

				m = ((gga->lon % 600000) / 10000.0);
				if (m < 0)
					m = -m;

				snprintf(buf, 17, "%c%3d %.1f GT%3d", gga->lon < 0 ? 'W' : 'E', d, m, (int)vtg->magGTK);
				lcd.setCursorPos(1, 0);
				lcd.writeString(buf);

				gga->destroy(), gga = NULL;
				vtg->destroy(), vtg = NULL;
			}

			if (g == 0)
				g = serialDataAvail(fd); // keep the loop going if possible
		}

		usleep(DELAY);
	}

	return 0;
}
