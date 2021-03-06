#ifndef FlightDirector_hpp
#define FlightDirector_hpp

#include "Autopilot.hpp"
#include "DataSource.hpp"
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

public:
  FlightDirector(Autopilot *_ap, DataSource *_data, GISDatabase *_db, LogCallback _log);

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

  void updateHeadingTrackMode(unsigned int _elapsedMilliseconds, double _dis, double _brg);

  void updateHeadingCircleMode(unsigned int _elapsedMilliseconds, double _dis, double _brg);

private:
  Autopilot *ap;
  DataSource *data;
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
