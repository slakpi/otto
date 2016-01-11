#include <stdexcept>
#include "XPlaneDataSource.hpp"

XPlaneDataSource::XPlaneDataSource()
:	lat(nullptr),
	lon(nullptr),
	alt(nullptr),
	hdg(nullptr),
	gs(nullptr)
{
/*	Only use data that can be derived from a GPS. */
	
	lat = XPLMFindDataRef("sim/flightmodel/position/latitude");
	lon = XPLMFindDataRef("sim/flightmodel/position/longitude");
	alt = XPLMFindDataRef("sim/flightmodel/position/elevation");
	hdg = XPLMFindDataRef("sim/flightmodel/position/hpath"); //closest to a GPS ground track.
	gs = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
}

XPlaneDataSource::~XPlaneDataSource()
{
	
}

bool XPlaneDataSource::sample(Data *_data) const
{
	_data->lat = _data->lon = 0.0;
	_data->alt = 0.0;
	_data->hdg = 0.0;
	_data->gs = 0.0;
	
	if (lat == nullptr || lon == nullptr || alt == nullptr || hdg == nullptr || gs == nullptr)
		return false;

	_data->lat = XPLMGetDatad(lat);
	_data->lon = XPLMGetDatad(lon);
	_data->alt = XPLMGetDatad(alt) * 3.28084; //meters -> feet
	_data->hdg = XPLMGetDataf(hdg);
	_data->gs = XPLMGetDataf(gs) * 1.94384; //meters per second -> knots
	
	return true;
}