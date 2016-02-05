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
	enum Mode
	{
		seekMode,
		trackMode,
		circleMode
	};
	
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
	void updateProjectedDistance(unsigned int _elapsedMilliseconds);
	
	void updateProjectedLandingPoint(unsigned int _elapsedMilliseconds);
	
	void updateHeading(unsigned int _elapsedMilliseconds);
	
	void updateHeadingSeekMode(unsigned int _elapsedMilliseconds);
	
	void updateHeadingTrackMode(unsigned int _elapsedMilliseconds, double dis, double brg);
	
	void updateHeadingCircleMode(unsigned int _elapsedMilliseconds, double dis, double brg);
	
private:
	Autopilot *ap;
	DataSource *data;
	TimerSource *timer;
	GISDatabase *db;
	LogCallback log;
	Mode mode;
	Data lastSample;
	Loc projLoc;
	double projDistance, targetHdg;
	AveragingBuffer rateOfTurn;
	AveragingBuffer verticalSpeed;
	AveragingBuffer groundSpeed;
	Loc originLoc;
	RecoveryLocation recoveryLoc;
	double recoveryCourse;
	unsigned int seekCourseTime;
};

#endif