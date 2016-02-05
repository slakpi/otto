#include <stdexcept>
#include <string>
#include <cstring>
#include <cfloat>
#include "FlightDirector.hpp"
#include "GISDatabase.hpp"
#include "Utilities.hpp"

static const unsigned int rateOfTurnDelay = 2;
static const unsigned int verticalSpeedDelay = 5;
static const unsigned int groundSpeedDelay = 5;

static const double maxHdgErr = 30.0;
static const double maxRoT = 3.0;
static const double minAltAGL = 5000.0;

static inline double maxCircleDistance(double _gs)
{
/*	10 nm = 2 minutes at 300 kts ground speed.
	1.6 nm ~ 2 minutes at 50 kts ground speed.
 */
	
	return (min(max(_gs, 50.0), 300.0) - 50) * ((10 - 1.6) / 250.0) + 1.6;
}

void FlightDirector::timerCallback(double _interval, void *_arg)
{
	FlightDirector *fd = static_cast<FlightDirector*>(_arg);

	if (fd == nullptr)
		return;

/*	clamp the lower bound of `_interval' to zero and convert it to milliseconds
	using normal rounding.
 */

	fd->refresh((unsigned int)(max(_interval, 0.0) * 1000 + 0.5));
}

FlightDirector::FlightDirector(Autopilot *_ap, DataSource *_data, TimerSource *_timer, GISDatabase *_db, LogCallback _log)
:	ap(_ap),
	data(_data),
	timer(_timer),
	db(_db),
	log(_log),
	mode(seekMode),
	projDistance(0),
	targetHdg(0),
	rateOfTurn(rateOfTurnDelay),
	verticalSpeed(verticalSpeedDelay),
	groundSpeed(groundSpeedDelay),
	recoveryCourse(0),
	seekCourseTime(0)
{
	if (ap == nullptr)
		throw std::invalid_argument("_ap");
	if (data == nullptr)
		throw std::invalid_argument("_data");
	if (timer == nullptr)
		throw std::invalid_argument("_timer");
	if (db == nullptr)
		throw std::invalid_argument("_db");
	if (log == nullptr)
		throw std::invalid_argument("_log");

	memset(&projLoc, 0, sizeof(projLoc));
	memset(&recoveryLoc, 0, sizeof(recoveryLoc));
	memset(&originLoc, 0, sizeof(originLoc));

	timer->setCallback(timerCallback, this);
}

FlightDirector::~FlightDirector()
{
	delete ap;
	delete data;
	delete timer;
	delete db;
}

void FlightDirector::enable()
{
	timer->setTimer(1000);
}

void FlightDirector::disable()
{
	timer->killTimer();
}

void FlightDirector::refresh(unsigned int _elapsedMilliseconds)
{
/*	Ra = Actual rate-of-turn derived from an averaged rate of GPS heading change.
	Rt = Target rate-of-turn calculated from the response curve below.
	dR = Rate-of-turn error (Rt - Ra).
	dH = Error between GPS heading and target heading.
	Ar = Rudder angle calculated from the response curve below.
 */

	Data d;
	double Ra, Rt, dR, dH, Ar;

	if (!data->sample(&d))
		return;
	if (_elapsedMilliseconds < 1)
		return; /* divide by zero protection. */

	Ra = rateOfTurn.pushSample((d.hdg - lastSample.hdg) * 1000 / _elapsedMilliseconds);
	verticalSpeed.pushSample((d.alt - lastSample.alt) * 1000 / _elapsedMilliseconds * 60);
	groundSpeed.pushSample(d.gs);
	lastSample = d;

	updateProjectedDistance(_elapsedMilliseconds);
	updateHeading(_elapsedMilliseconds);

/*	the target rate-of-turn follows an exponential curve designed to hit +/- 3 degrees per
	second at a heading error of +/- 30 degrees.  an exponential curve gives us a reponse
	that is initially shallow and steepens as the heading error increases.  the shallow
	initial response prevents overcorrection for small heading errors.

						  |dH|
	  Rt = ( 1.0472941228      - 1 ) * sgn( dH )

	the rudder angle follows a logarithmic curve with a steeper response when the delta
	between the target rate of turn and the actual rate of turn approaches zero.  the
	logarithmic curve hits +/- 1 units of rudder deflection at the 3 degree per second
	maximum rate of turn. dR is positive for right deflection and negative for left
	deflection.

	  Ar = ( log10( |dR| + .33 ) + .48 ) * sgn( dR )
 */

	dH = fmod(fmod(targetHdg - d.hdg, 360.0) + 540.0, 360.0) - 180.0;
	Rt = min(pow(1.0472941228, min(fabs(dH), maxHdgErr)) - 1, maxRoT) * sgn(dH);
	dR = Rt - Ra;
	Ar = min(log10(min(fabs(dR), maxRoT) + 0.33) + 0.48, 1.0) * sgn(dR);

	ap->setRudderDeflection((float)Ar);
}

void FlightDirector::updateProjectedDistance(unsigned int _elapsedMilliseconds)
{
	double av = verticalSpeed.average();
	double ag = max(groundSpeed.average(), 0.0);
	double agl = lastSample.alt;

/*	assume a nominal -1 ft/s if the average vertical speed is greater than -1 ft/s.  this
	both protects from division by zero and effectively assumes level flight if the glider
	is climbing.  we can recompute when the glider resumes a descent.

	clamp ground speed to positive values.
 
	clamp distance to 3,000 nm.  this keeps the projections from getting silly.
 */

	av = (av > -1 ? -1 : av);

	if (recoveryLoc.id != -1)
		agl -= recoveryLoc.elev;
	
	projDistance = min(agl / (-av * 60) * ag, 3000.0);
}

void FlightDirector::updateProjectedLandingPoint(unsigned int _elapsedMilliseconds)
{
/*	the heading should be a true ground track so that we are taking winds into account. */

	getDestination(lastSample.pos, lastSample.hdg, projDistance, projLoc);
}

void FlightDirector::updateHeading(unsigned int _elapsedMilliseconds)
{
	double dis = 0, brg = 0;
	
	if (mode == trackMode || mode == circleMode)
	{
		getDistanceAndBearing(lastSample.pos, recoveryLoc.pos, dis, brg);
		
		if (dis > projDistance && lastSample.alt - recoveryLoc.elev > minAltAGL)
		{
			
/*	if the distance to the current recovery point is greater than our projected glide
	distance, go back into seek mode.  UNLESS we are below 5000 feet AGL.  in that
	case, just keep heading toward the recovery location.
 */
			
			mode = seekMode;
			recoveryLoc.id = -1;
			seekCourseTime = 0;
			(*log)("OTTO: no longer able to make %s, entering seek mode.\n", recoveryLoc.ident.c_str());
		}
	}
	
	switch (mode)
	{
		case seekMode:
			updateHeadingSeekMode(_elapsedMilliseconds);
			break;
		case trackMode:
			updateHeadingTrackMode(_elapsedMilliseconds, dis, brg);
			break;
		case circleMode:
			updateHeadingCircleMode(_elapsedMilliseconds, dis, brg);
			break;
	}
}

void FlightDirector::updateHeadingSeekMode(unsigned int _elapsedMilliseconds)
{
	RecoveryLocation loc;
	double dis, brg;
	
	if (db->getRecoveryLocation(lastSample.pos, lastSample.hdg, projDistance, loc))
	{
		getDistanceAndBearing(lastSample.pos, loc.pos, dis, brg);
		
		mode = trackMode;
		recoveryLoc = loc;
		originLoc = lastSample.pos;
		recoveryCourse = brg;
		(*log)("OTTO: tracking to %s (elev. %.1f) on a course of %.0f.\n", loc.ident.c_str(), loc.elev, brg);
	}
	else
	{
		
/*	fly a box pattern with 2 minute legs. */
		
		seekCourseTime += _elapsedMilliseconds;
		
		if (seekCourseTime >= 120000)
		{
			targetHdg = fmod(targetHdg + 90.0, 360.0);
			seekCourseTime = 0;
		}
	}
}

void FlightDirector::updateHeadingTrackMode(unsigned int _elapsedMilliseconds, double dis, double brg)
{
	double x, a, ag = groundSpeed.average();
	
/*	use a linear scale to determine when to enter circle mode.  at 50 kts or less, circle
	at 1 nm or less.  at 500 kts or more, circle at 10 nm or less.
 */
	
	if (dis <= maxCircleDistance(ag))
	{
		mode = circleMode;
		(*log)("OTTO: entering circle mode around %s.\n", recoveryLoc.ident.c_str());
		
		return;
	}
	
/*	calculate the cross-track error, then use a linear forumla to calculate an intercept
	correction based on the amount of time, in minutes, required to cover the cross-track
	error distance.
 
	       x * 60     90 degrees
	  a = -------- * ------------
	         GS        2 minutes

	cross-track error is negative when left of course and positive when right of course,
	so subtract the intercept correction.
 */
 
	x = crossTrackError(originLoc, recoveryLoc.pos, lastSample.pos);
	a = min(x * 60.0 / ag * 45.0, 90.0);
	targetHdg = fmod(fmod(recoveryCourse - a, 360.0) + 360.0, 360.0);
}

void FlightDirector::updateHeadingCircleMode(unsigned int _elapsedMilliseconds, double dis, double brg)
{
	if (dis > maxCircleDistance(groundSpeed.average()))
	{
		mode = trackMode;
		recoveryCourse = brg;
		(*log)("OTTO: entering track mode to %s on a new course of %.0f.\n", recoveryLoc.ident.c_str(), brg);
		
		return;
	}
	
	targetHdg = brg;
}