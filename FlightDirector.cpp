#include <stdexcept>
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
	projDistance(0),
	targetHdg(0),
	rateOfTurn(rateOfTurnDelay),
	verticalSpeed(verticalSpeedDelay),
	groundSpeed(groundSpeedDelay),
	curRecoveryLoc(-1),
	recoveryCourse(0)
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
/*	Ra = Actual rate-of-turn derived from an averaged rate of "GPS" heading change.
	Rt = Target rate-of-turn calculated from the response curve below.
	dR = Rate-of-turn error (Rt - Ra).
	dH = Error between "GPS" heading and target heading.
	Ar = Rudder angle calculated from the response curve below.
 */
	
	Data d;
	double av, Ra, Rt, dR, dH, Ar;

	if (!data->sample(&d))
		return;
	if (_elapsedMilliseconds < 1)
		return; /* divide by zero protection. */

	Ra = rateOfTurn.pushSample((d.hdg - lastSample.hdg) * 1000 / _elapsedMilliseconds);
	av = verticalSpeed.pushSample((d.alt - lastSample.alt) * 1000 / _elapsedMilliseconds * 60);
	groundSpeed.pushSample(d.gs);
	lastSample = d;
		
	updateProjectedDistance();
//	updateProjectedLandingPoint();
	updateTargetHeading();

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

void FlightDirector::updateProjectedDistance()
{
	double av = verticalSpeed.average();
	double ag = max(groundSpeed.average(), 0.0);
	
/*	assume a nominal -1 ft/s if the average vertical speed is greater than -1 ft/s.  this
	both protects from division by zero and effectively assumes level flight if the glider
	is climbing.  we can recompute when the glider resumes a descent.
 
	clamp ground speed to positive values.
	 
	clamp distance to 3,000 nm.  this keeps the projections from getting silly.
 */
	
	av = (av > -1 ? -1 : av);
	
	projDistance = min(lastSample.alt / (-av * 60) * ag, 3000.0);
}

void FlightDirector::updateProjectedLandingPoint()
{
/*	the heading should be a true ground track so that we are taking winds into account. */
	
	getDestination(lastSample.pos, lastSample.hdg, projDistance, projLoc);
}

void FlightDirector::updateTargetHeading()
{
	int64_t r;
	Loc l;
	double d, b;
	
	targetHdg = 180.0;
	
	if (!db->getRecoveryLocation(lastSample.pos, lastSample.hdg, projDistance, r, l))
	{
		if (curRecoveryLoc != -1)
			(*log)("OTTO no longer has the glide distance to reach a recovery location.  Tracking 180.\n");
		
		curRecoveryLoc = -1;
	}
	else
	{
		if (r != curRecoveryLoc)
		{
			curRecoveryLoc = r;
			recoveryLoc = l;
			(*log)("OTTO is homing to recovery location %lld.\n", r);
		}
		
		getDistanceAndBearing(lastSample.pos, l, d, b);
		
/*	this is simple homing.  build tracking logic in. */
		
		targetHdg = b;
		recoveryCourse = b;
	}
}