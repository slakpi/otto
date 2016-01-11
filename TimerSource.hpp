#ifndef TimerSource_hpp
#define TimerSource_hpp

/*******************************************************************************

 The TimerSource class establishes an interface for timing.
 
 ******************************************************************************/

class TimerSource
{
public:
	typedef void (*TimerCallback)(double, void*);
	
public:
	TimerSource();
	
	TimerSource(TimerCallback _callback, void *_callbackArg);
	
public:
	virtual ~TimerSource();
	
public:
	void setCallback(TimerCallback _callback, void *_arg);

public:
	virtual void setTimer(unsigned int _milliseconds) = 0;
	
	virtual void pauseTimer() = 0;
	
	virtual void resumeTimer() = 0;
	
	virtual void killTimer() = 0;
	
protected:
	TimerCallback callback;
	void *callbackArg;
};

#endif