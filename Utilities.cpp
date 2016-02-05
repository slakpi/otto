#include "Utilities.hpp"

static const double R = 3440.277; //mean Earth radius in NM.

void getDestination(const Loc &_pos, double _hdg, double _distance, Loc &_dest)
{
	Loc p;
	double h;
	
	p.lat = degToRad(_pos.lat);
	p.lon = degToRad(_pos.lon);
	h = degToRad(_hdg);
	
	_dest.lat = asin(sin(p.lat) * cos(_distance / R) + cos(p.lat) * sin(_distance / R) * cos(h));
	_dest.lon = p.lon + atan2(sin(h) * sin(_distance / R) * cos(p.lat), cos(_distance / R) - sin(p.lat) * sin(_dest.lat));
	
	_dest.lat = max(min(radToDeg(_dest.lat), 90.0), -90.0);
	_dest.lon = max(min(radToDeg(_dest.lon), 180.0), -180.0);
}

static void _getDistanceAndBearing(const Loc &_pos1, const Loc &_pos2, double &_distance, double &_bearing)
{
	Loc p1, p2;
	double dLat, dLon, x, y;
	
	p1.lat = degToRad(_pos1.lat);
	p1.lon = degToRad(_pos1.lon);
	p2.lat = degToRad(_pos2.lat);
	p2.lon = degToRad(_pos2.lon);
	
	dLat = p2.lat - p1.lat;
	dLon = p2.lon - p1.lon;
	x = sin(dLat / 2) * sin(dLat / 2) + cos(p1.lat) * cos(p2.lat) * sin(dLon / 2) * sin(dLon / 2);
	_distance = 2 * atan2(sqrt(x), sqrt(1 - x));
	
	x = sin(dLon) * cos(p2.lat);
	y = cos(p1.lat) * sin(p2.lat) - sin(p1.lat) * cos(p2.lat) * cos(dLon);
	_bearing = atan2(x, y);
}

void getDistanceAndBearing(const Loc &_pos1, const Loc &_pos2, double &_distance, double &_bearing)
{
	double d, t;
	_getDistanceAndBearing(_pos1, _pos2, d, t);
	_distance = R * d;
	_bearing = radToDeg(t);
	
/*	_getDistanceAndBearing() outputs a bearing in the range (-PI, PI].  after converting to
	degrees, normalize the degrees to [0, 360).
 */
	
	_bearing = (_bearing < 0.0 ? _bearing + 360.0 : _bearing);
}

double crossTrackError(const Loc &_origin, const Loc &_dest, const Loc &_ppos)
{
	double d12, d13, t12, t13;
	
	_getDistanceAndBearing(_origin, _dest, d12, t12);
	_getDistanceAndBearing(_origin, _ppos, d13, t13);
	
	return asin(sin(d13) * sin(t13 - t12)) * R;
}