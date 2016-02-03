#include <stdexcept>
#include "RpiTimerSource.hpp"

RpiTimerSource::RpiTimerSource()
{

}

RpiTimerSource::RpiTimerSource(TimerCallback _callback, void *_callbackArg)
:	TimerSource(_callback, _callbackArg)
{

}

RpiTimerSource::~RpiTimerSource()
{
	killTimer();
}

void RpiTimerSource::setTimer(unsigned int _milliseconds)
{

}

void RpiTimerSource::pauseTimer()
{

}

void RpiTimerSource::resumeTimer()
{

}

void RpiTimerSource::killTimer()
{

}
