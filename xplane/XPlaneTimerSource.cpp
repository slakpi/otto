#include <stdexcept>
#include "XPlaneTimerSource.hpp"

float XPlaneTimerSource::flightLoopCallback(float _elapsedSinceLastCall, float _elapsedSinceLastFlightLoop, int _counter, void *_arg)
{
	XPlaneTimerSource *timer = static_cast<XPlaneTimerSource*>(_arg);
	
	if (timer == nullptr || timer->callback == nullptr)
		return 0.0f; /* just disable the timer; we can't do anything else. */
	
	(*timer->callback)(_elapsedSinceLastCall, timer->callbackArg);
	
	return timer->reqInterval;
}

XPlaneTimerSource::XPlaneTimerSource()
{
	
}

XPlaneTimerSource::XPlaneTimerSource(TimerCallback _callback, void *_callbackArg)
:	TimerSource(_callback, _callbackArg),
	reqInterval(0.0f)
{
	
}

XPlaneTimerSource::~XPlaneTimerSource()
{
	killTimer();
}

void XPlaneTimerSource::setTimer(unsigned int _milliseconds)
{

/*******************************************************************************

 NOTE: Because X-Plane is a simulator governed by framerate (i.e., it cannot run
 faster than it can render graphics for the current frame), `_milliseconds' will
 have its lower bound (maximum speed) clamped at the simulator framerate.

 ******************************************************************************/
	
	reqInterval = (float)_milliseconds / 1000.0f;
	XPLMRegisterFlightLoopCallback(flightLoopCallback, reqInterval, this);
}

void XPlaneTimerSource::pauseTimer()
{
	XPLMSetFlightLoopCallbackInterval(flightLoopCallback, 0.0f, 0, this);
}

void XPlaneTimerSource::resumeTimer()
{
	XPLMSetFlightLoopCallbackInterval(flightLoopCallback, reqInterval, 0, this);
}

void XPlaneTimerSource::killTimer()
{
	XPLMUnregisterFlightLoopCallback(flightLoopCallback, this);
	reqInterval = 0.0f;
}