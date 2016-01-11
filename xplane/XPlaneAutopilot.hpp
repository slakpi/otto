#ifndef XPlaneAutopilot_hpp
#define XPlaneAutopilot_hpp

#include "Autopilot.hpp"

/*******************************************************************************
 
 The XPlaneAutopilot class carries out FlightDirector instructions using the
 X-Plane API to affect the current simulated aircraft.
 
 ******************************************************************************/

class XPlaneAutopilot : public Autopilot
{
public:
	XPlaneAutopilot();
	
public:
	virtual ~XPlaneAutopilot();
};

#endif