#ifndef RpiTimerSource_hpp
#define RpiTimerSource_hpp

#include <TimerSource.hpp>

/*******************************************************************************

 ******************************************************************************/

class RpiTimerSource : public TimerSource
{
public:
	RpiTimerSource();

	RpiTimerSource(TimerCallback _callback, void *_callbackArg);

public:
	virtual ~RpiTimerSource();

public:
	virtual void setTimer(unsigned int _milliseconds);

	virtual void pauseTimer();

	virtual void resumeTimer();

	virtual void killTimer();
};

#endif
