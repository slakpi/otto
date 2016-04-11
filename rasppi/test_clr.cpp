#include <iostream>
#include <wiringPi.h>
#include "HD44780.hpp"

using namespace std;

int main(int _argc, char* _argv[])
{
	HD44780 lcd;

	if (wiringPiSetup() == -1)
	{
		cerr << "Failed to initialize wiringPi.\n";
		return -1;
	}

	if (!lcd.init())
	{
		cerr << "Failed to initialize LCD driver.\n";
		return -1;
	}

	lcd.clear();

	return 0;
}
