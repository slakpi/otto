#ifndef FlightDirector_hpp
#define FlightDirector_hpp

#include "Autopilot.hpp"
#include "DataSource.hpp"
#include "TimerSource.hpp"
#include "GISDatabase.hpp"
#include "AveragingBuffer.hpp"

typedef void (*LogCallback)(const char *_fmt, ...);

class FlightDirector
{
private:
	static void timerCallback(double _interval, void *_arg);
	
public:
	FlightDirector(Autopilot *_ap, DataSource *_data, TimerSource *_timer, GISDatabase *_db, LogCallback _log);
	
public:
	~FlightDirector();
	
public:
	void enable();
	
	void disable();
	
	void refresh(unsigned int _elapsedMilliseconds);
	
private:
	void updateProjectedDistance();
	
	void updateProjectedLandingPoint();
	
	void updateTargetHeading();
	
private:
	Autopilot *ap;
	DataSource *data;
	TimerSource *timer;
	GISDatabase *db;
	LogCallback log;
	Data lastSample;
	Loc projLoc;
	double projDistance, targetHdg;
	AveragingBuffer rateOfTurn;
	AveragingBuffer verticalSpeed;
	AveragingBuffer groundSpeed;
	int64_t curRecoveryLoc;
	Loc recoveryLoc;
	double recoveryCourse;
};

#endif