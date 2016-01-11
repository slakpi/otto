#ifndef XPlaneTimerSource_hpp
#define XPlaneTimerSource_hpp

#include <XPLM/XPLMProcessing.h>
#include "TimerSource.hpp"

/*******************************************************************************

 The XPlaneTimerSource uses the X-Plane API to provide timing.
 
 ******************************************************************************/

class XPlaneTimerSource : public TimerSource
{
private:
	static float flightLoopCallback(float _elapsedSinceLastCall, float _elapsedSinceLastFlightLoop, int _counter, void *_arg);
	
public:
	XPlaneTimerSource();
	
	XPlaneTimerSource(TimerCallback _callback, void *_callbackArg);
	
public:
	virtual ~XPlaneTimerSource();
	
public:
	virtual void setTimer(unsigned int _milliseconds);
	
	virtual void pauseTimer();
	
	virtual void resumeTimer();
	
	virtual void killTimer();
	
private:
	float reqInterval;
};

#endif