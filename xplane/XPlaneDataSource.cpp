#include <stdexcept>
#include "XPlaneDataSource.hpp"

XPlaneDataSource::XPlaneDataSource()
:	latRef(nullptr),
	lonRef(nullptr),
	altRef(nullptr),
	hdgRef(nullptr),
	gsRef(nullptr)
{
/*	Only use data that can be derived from a GPS. */
	
	latRef = XPLMFindDataRef("sim/flightmodel/position/latitude");
	lonRef = XPLMFindDataRef("sim/flightmodel/position/longitude");
	altRef = XPLMFindDataRef("sim/flightmodel/position/elevation");
	hdgRef = XPLMFindDataRef("sim/flightmodel/position/hpath"); //closest to a GPS ground track.
	gsRef = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
}

XPlaneDataSource::~XPlaneDataSource()
{
	
}

bool XPlaneDataSource::sample(Data *_data) const
{
	_data->pos.lat = _data->pos.lon = 0.0;
	_data->alt = 0.0;
	_data->hdg = 0.0;
	_data->gs = 0.0;
	
	if (latRef == nullptr || lonRef == nullptr || altRef == nullptr || hdgRef == nullptr || gsRef == nullptr)
		return false;

	_data->pos.lat = XPLMGetDatad(latRef);
	_data->pos.lon = XPLMGetDatad(lonRef);
	_data->alt = XPLMGetDatad(altRef) * 3.28084; //meters -> feet
	_data->hdg = XPLMGetDataf(hdgRef);
	_data->gs = XPLMGetDataf(gsRef) * 1.94384; //meters per second -> knots
	
	return true;
}