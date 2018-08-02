#include <stdexcept>
#include <algorithm>
#include <string>
#include <cstring>
#include <cfloat>
#include "FlightDirector.hpp"
#include "GISDatabase.hpp"
#include "Utilities.hpp"

using namespace std;

static const unsigned int rateOfTurnDelay = 2;
static const unsigned int verticalSpeedDelay = 5;
static const unsigned int groundSpeedDelay = 5;

static const double maxHdgErr = 30.0;
static const double maxRoT = 3.0;
static const double minAltAGL = 5000.0;

static inline double interceptCorrection(double _hdg, double _err, double _gs)
{

/*	90 degree correction when 2 minutes off course. */

	return fmod(fmod(_hdg - min(_err * 60.0 / _gs * 45.0, 90.0), 360.0) + 360.0, 360.0);
}

static inline double maxCircleDistance(double _projDistance)
{

/*	tighten the circle as projected distance decreases.  circle at a maximum of
	5 nm and a minimum of 1 nm.
 */

	return max(min(_projDistance / 2.0 - 1.0, 5.0), 1.0);
}

FlightDirector::FlightDirector(Autopilot *_ap, DataSource *_data, GISDatabase *_db, LogCallback _log)
:	ap(_ap),
	data(_data),
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
	if (db == nullptr)
		throw std::invalid_argument("_db");
	if (log == nullptr)
		throw std::invalid_argument("_log");

	memset(&projLoc, 0, sizeof(projLoc));
	memset(&recoveryLoc, 0, sizeof(recoveryLoc));
	memset(&originLoc, 0, sizeof(originLoc));
}

FlightDirector::~FlightDirector()
{
	delete ap;
	delete data;
	delete db;
}

void FlightDirector::enable()
{
	ap->enable();
}

void FlightDirector::disable()
{
	ap->disable();
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
 */

	dH = fmod(fmod(targetHdg - d.hdg, 360.0) + 540.0, 360.0) - 180.0;
	Rt = min(pow(1.0472941228, min(fabs(dH), maxHdgErr)) - 1, maxRoT) * sgn(dH);

	if (d.avail & DATA_ROLL)
/*	reduce the target rate-of-turn if we have excessive bank. */
		Rt -= (max(fabs(d.roll), 30.0) - 30.0) / 60.0 * sgn(d.roll);

/*	the rudder angle follows a logarithmic curve with a steeper response when the delta
	between the target rate of turn and the actual rate of turn approaches zero.  the
	logarithmic curve hits +/- 1 units of rudder deflection at the 3 degree per second
	maximum rate of turn. dR is positive for right deflection and negative for left
	deflection.

	  Ar = ( log10( |dR| + .33 ) + .48 ) * sgn( dR )
 */

	dR = Rt - Ra;
	Ar = min(log10(min(fabs(dR), maxRoT) + 0.33) + 0.48, 1.0) * sgn(dR);

	if (d.avail & DATA_PITCH)
	{
		if (d.pitch < -20.0 || d.pitch > 20.0)
/*	if the pitch is too excessive, just center the rudder to prevent spins. */
			Ar = 0.0f;
	}

	ap->setRudderDeflection((float)Ar);
}

void FlightDirector::updateProjectedDistance(unsigned int _elapsedMilliseconds)
{

/*	assume a nominal -1 ft/s if the average vertical speed is greater than -1 ft/s.  this
	both protects from division by zero and effectively assumes level flight if the glider
	is climbing.  we can recompute when the glider resumes a descent.

	clamp ground speed to positive values.

	clamp distance to 3,000 nm.  this keeps the projections from getting silly.
 */

	double av = min(verticalSpeed.average(), -1.0);
	double ag = max(groundSpeed.average(), 0.0);
	double agl = lastSample.alt;

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

	getDistanceAndBearing(lastSample.pos, recoveryLoc.pos, dis, brg);

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
		(*log)("OTTO: tracking to %s (elev. %.1f) on a course of %.0f.\n",
			loc.ident,
			loc.elev,
			brg);
	}
	else
	{

/*	fly a box pattern with 1 minute legs. */

		seekCourseTime += _elapsedMilliseconds;

		if (seekCourseTime >= 60000)
		{
			targetHdg = fmod(targetHdg + 90.0, 360.0);
			seekCourseTime = 0;
		}
	}
}

void FlightDirector::updateHeadingTrackMode(unsigned int _elapsedMilliseconds, double _dis, double _brg)
{
	double x, ag = groundSpeed.average(), md = maxCircleDistance(projDistance);

	if (_dis > projDistance && lastSample.alt - recoveryLoc.elev > minAltAGL)
	{

		/*	if the distance to the current recovery point is greater than our projected glide
		 distance, go back into seek mode.  UNLESS we are below 5000 feet AGL.  in that
		 case, just keep heading toward the recovery location.
		 */

		mode = seekMode;
		recoveryLoc.id = -1;
		seekCourseTime = 0;
		(*log)("OTTO: no longer able to make %s, entering seek mode.\n", recoveryLoc.ident);
	}

	if (_dis <= md + 2.0)
	{
		mode = circleMode;
		(*log)("OTTO: entering circle mode around %s.\n", recoveryLoc.ident);

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
	targetHdg = interceptCorrection(recoveryCourse, x, ag);
}

void FlightDirector::updateHeadingCircleMode(unsigned int _elapsedMilliseconds, double _dis, double _brg)
{
	double ag = groundSpeed.average(), md = maxCircleDistance(projDistance);

	if (_dis > md + 5.0)
	{
		mode = trackMode;
		recoveryCourse = _brg;
		(*log)("OTTO: entering track mode to %s on a new course of %.0f.\n",
			recoveryLoc.ident,
			_brg);

		return;
	}

/*	maintain a constant distance circle around the recovery location. */

	targetHdg = interceptCorrection(_brg + 90.0, _dis - md, ag);
}
