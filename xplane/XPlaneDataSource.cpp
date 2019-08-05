#include <stdexcept>
#include "XPlaneDataSource.hpp"

XPlaneDataSource::XPlaneDataSource()
: latRef(nullptr),
  lonRef(nullptr),
  altRef(nullptr),
  hdgRef(nullptr),
  gsRef(nullptr),
  pitchRef(nullptr),
  rollRef(nullptr),
  yawRef(nullptr)
{
  latRef = XPLMFindDataRef("sim/flightmodel/position/latitude");
  lonRef = XPLMFindDataRef("sim/flightmodel/position/longitude");
  altRef = XPLMFindDataRef("sim/flightmodel/position/elevation");
  hdgRef = XPLMFindDataRef("sim/flightmodel/position/hpath"); //closest to a GPS ground track.
  gsRef = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
  pitchRef = XPLMFindDataRef("sim/flightmodel/position/true_theta");
  rollRef = XPLMFindDataRef("sim/flightmodel/position/true_phi");
  yawRef = XPLMFindDataRef("sim/flightmodel/position/true_psi");
}

XPlaneDataSource::~XPlaneDataSource()
{

}

bool XPlaneDataSource::sample(Data *_data) const
{
  _data->avail = 0;
  _data->pos.lat = _data->pos.lon = 0.0;
  _data->alt = 0.0;
  _data->hdg = 0.0;
  _data->gs = 0.0;
  _data->pitch = 0.0;
  _data->roll = 0.0;
  _data->yaw = 0.0;

  if (latRef != nullptr && lonRef != nullptr)
  {
    _data->avail |= DATA_POS;
    _data->pos.lat = XPLMGetDatad(latRef);
    _data->pos.lon = XPLMGetDatad(lonRef);
  }

  if (altRef != nullptr)
  {
    _data->avail |= DATA_ALT;
    _data->alt = XPLMGetDatad(altRef) * 3.28084; //meters -> feet
  }

  if (hdgRef != nullptr)
  {
    _data->avail |= DATA_HDG;
    _data->hdg = XPLMGetDataf(hdgRef);
  }

  if (gsRef != nullptr)
  {
    _data->avail |= DATA_GS;
    _data->gs = XPLMGetDataf(gsRef) * 1.94384; //meters per second -> knots
  }

  if (pitchRef != nullptr)
  {
    _data->avail |= DATA_PITCH;
    _data->pitch = XPLMGetDataf(pitchRef);
  }

  if (rollRef != nullptr)
  {
    _data->avail |= DATA_ROLL;
    _data->roll = XPLMGetDataf(rollRef);
  }

  if (yawRef != nullptr)
  {
    _data->avail |= DATA_YAW;
    _data->yaw = XPLMGetDataf(yawRef);
  }

  return true;
}
