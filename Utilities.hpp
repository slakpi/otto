#ifndef Utilities_hpp
#define Utilities_hpp

#include <cmath>

template<class T>
T min(const T &_a, const T &_b)
{
	return (_a < _b ? _a : _b);
}

template<class T>
T max(const T &_a, const T &_b)
{
	return (_a > _b ? _a : _b);
}

inline double degToRad(double _deg)
{
	return (_deg * M_PI / 180.0);
}

inline double radToDeg(double _rad)
{
	return (_rad * 180.0 / M_PI);
}

#define COUNTOF(a) (sizeof(a) / sizeof((a)[0]))

#endif