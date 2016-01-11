#include <stdexcept>
#include "TimerSource.hpp"

TimerSource::TimerSource()
:	callback(nullptr),
	callbackArg(nullptr)
{
	
}

TimerSource::TimerSource(TimerCallback _callback, void *_callbackArg)
:	callback(_callback),
	callbackArg(_callbackArg)
{

}

TimerSource::~TimerSource()
{
	
}

void TimerSource::setCallback(TimerCallback _callback, void *_arg)
{
	pauseTimer();
	callback = _callback;
	callbackArg = _arg;
	resumeTimer();
}