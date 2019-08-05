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
#include "RpiDataSource.hpp"
#include "HD44780.hpp"

#define DELAY     1000000

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
  HD44780 lcd;
  RpiDataSource data;
  Data sample;
  float pos = 0.0f, d = 0.25f;
  char str[17];

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

  if (!lcd.init())
  {
    cerr << "Failed to initialize LCD.\n";
    return -1;
  }

  if (!data.start())
  {
    cerr << "Failed to initialize data source.\n";
    return -1;
  }

  while (run)
  {
    data.sample(&sample);
    a.setServoPos(pos);

    lcd.clear();
    lcd.setCursorPos(0, 0);
    snprintf(str, 16, "%-2.2f* %-3.2f*", sample.pos.lat, sample.pos.lon);
    lcd.writeString(str);

    lcd.setCursorPos(1, 0);
    snprintf(str, 16, "%-1.2f", a.getServoPos());
    lcd.writeString(str);

    if (pos >= 1.0f)
      d = -0.25;
    else if (pos <= -1.0f)
      d = 0.25;

    pos += d;

    usleep(DELAY);
  }

  return 0;
}
