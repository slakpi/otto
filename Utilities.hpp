#ifndef Utilities_hpp
#define Utilities_hpp

#include <cmath>
#include <algorithm>

template<typename T>
T clamp(const T &_a, const T &_l, const T &_h)
{
  return std::min(std::max(_a, _l), _h);
}

template<typename T> int sgn(const T &_val)
{
  return (T(0) < _val) - (_val < T(0));
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

struct Loc
{
  double lat;
  double lon;
};

void getDestination(const Loc &_pos, double _hdg, double _distance, Loc &_dest);

void getDistanceAndBearing(const Loc &_pos1, const Loc &_pos2, double &_distance, double &_bearing);

double crossTrackError(const Loc &_origin, const Loc &_dest, const Loc &_ppos);

#endif
