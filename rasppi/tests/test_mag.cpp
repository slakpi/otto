#include <sys/types.h>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <ctime>
#include <cmath>
#include <cfloat>
#include <unistd.h>
#include <wiringPi.h>
#include "LIS3MDL.hpp"

#define DELAY 9612

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
  LIS3MDL imuMag;
  DVector m;
  int64_t t, r;
  timespec spec;

  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  if (wiringPiSetup() == -1)
  {
    cerr << "Failed to initialize wiringPi.\n";
    return -1;
  }

  if (!imuMag.init())
  {
    cerr << "Failed to initialize IMU magnetometer.\n";
    return -1;
  }

  clock_gettime(CLOCK_MONOTONIC, &spec);
  r = spec.tv_sec * 1000000LL;
  r += spec.tv_nsec / 1000LL;

  cout << "t,Mx,My,Mz\n";

  while (run)
  {
    clock_gettime(CLOCK_MONOTONIC, &spec);
    t = spec.tv_sec * 1000000LL;
    t += spec.tv_nsec / 1000LL;
    t -= r;

    imuMag.readMag(m);

    cout << t << "," << m.x << "," << m.y << "," << m.z << endl;

    usleep(DELAY);
  }

  return 0;
}
