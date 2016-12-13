#include <sys/types.h>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include <wiringPi.h>
#include <AveragingBuffer.hpp>
#include "Arduino.hpp"

#define DELAY			1000000

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
	Arduino a;
	bool b = false;

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	if (wiringPiSetup() == -1)
	{
		cerr << "Failed to initialize wiringPi.\n";
		return -1;
	}

	if (!a.init())
	{
		cerr << "Failed to initialize Arduino.\n";
		return -1;
	}

	while (run)
	{
		a.setLight(b = !b);
		usleep(DELAY);
	}

	return 0;
}
