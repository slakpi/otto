#ifndef FlightDirector_hpp
#define FlightDirector_hpp

#include "Autopilot.hpp"
#include "DataSource.hpp"
#include "TimerSource.hpp"
#include "AveragingBuffer.hpp"

typedef void (*LogCallback)(const char *_fmt, ...);

class FlightDirector
{
private:
	enum
	{
		projectionTimer,
		timerCount // must be last item.
	};
	
	static const unsigned int verticalSpeedDelay = 5;
	static const unsigned int groundSpeedDelay = 5;
	static const unsigned int projectionInterval = 5000;
	
private:
	static void timerCallback(double _interval, void *_arg);
	
public:
	FlightDirector(Autopilot *_ap, DataSource *_data, TimerSource *_timer, LogCallback _log);
	
public:
	~FlightDirector();
	
public:
	void enable();
	
	void disable();
	
	void refresh(unsigned int _elapsedMilliseconds);
	
private:
	void updateProjectedLandingPoint();
	
private:
	Autopilot *ap;
	DataSource *data;
	TimerSource *timer;
	LogCallback log;
	Data lastSample;
	double projLat, projLon, projDistance;
	AveragingBuffer verticalSpeed;
	AveragingBuffer groundSpeed;
	unsigned int timers[timerCount];
};

#endif