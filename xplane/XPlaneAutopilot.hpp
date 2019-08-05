#ifndef XPlaneAutopilot_hpp
#define XPlaneAutopilot_hpp

#include <XPLM/XPLMDataAccess.h>
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
  
public:
  virtual void enable();
  
  virtual void disable();
  
  virtual float getRudderDeflection() const;
  
  virtual void setRudderDeflection(float _deflection);
  
private:
  XPLMDataRef fltCtrlOverrideRef;
  XPLMDataRef rudderDeflectionRef;
};

#endif