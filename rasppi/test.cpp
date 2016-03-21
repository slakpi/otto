#include <iostream>
#include <wiringPi.h>
#include "LSM6DS33.hpp"
#include "LIS3MDL.hpp"

using namespace std;

int main(int _argc, char* _argv[])
{
	LSM6DS33 lsm6;
	LIS3MDL lis3mdl;
	Vector<double> a, g, m;

	if (!lsm6.init())
		return -1;
	if (!lis3mdl.init())
		return -1;

	lsm6.readAccel(a);
	lsm6.readGyro(g);
	lis3mdl.readMag(m);

	cout << "(" << a.x << ", " << a.y << ", " << a.z << ")\n";
	cout << "(" << g.x << ", " << g.y << ", " << g.z << ")\n";
	cout << "(" << m.x << ", " << m.y << ", " << m.z << ")\n";

	return 0;
}
