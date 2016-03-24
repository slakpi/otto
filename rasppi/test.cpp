#include <iostream>
#include <csignal>
#include <cstdlib>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <termios.h>

#define BUF_SIZE 512

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
	struct termios options;
	int fd = -1, c = 0, g;
	char buf[BUF_SIZE + 1];

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	fd = serialOpen("/dev/ttyAMA0", 9600);

	tcgetattr(fd, &options);
	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;
	tcsetattr(fd, TCSANOW, &options);

	if (fd == -1)
	{
		cout << "Failed to open device.\n";
		return -1;
	}

	while (run)
	{
		g = serialDataAvail(fd);

		for ( ; g > 0; --g)
		{
			buf[c++] = (char)serialGetchar(fd);

			if (c == BUF_SIZE)
			{
				buf[c] = 0;
				cout << buf;
				c = 0;
				continue;
			}
		}

		if (c > 0)
		{
			buf[c] = 0;
			cout << buf << endl;
			c = 0;
		}

		sleep(1);
	}

	close(fd);

	return 0;
}
