#include <stdexcept>
#include <cstring>
#include "FlightDirector.hpp"
#include "Utilities.hpp"

static const unsigned int rateOfTurnDelay = 2;
static const unsigned int verticalSpeedDelay = 5;
static const unsigned int groundSpeedDelay = 5;
static const unsigned int projectionInterval = 5000;

static const double maxHdgErr = 30.0;
static const double maxRoT = 3.0;
static const double maxRudder = 0.25;

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

FlightDirector::FlightDirector(Autopilot *_ap, DataSource *_data, TimerSource *_timer, LogCallback _log)
:	ap(_ap),
	data(_data),
	timer(_timer),
	log(_log),
	projLat(0),
	projLon(0),
	projDistance(0),
	targetHdg(180.0),
	rateOfTurn(rateOfTurnDelay),
	verticalSpeed(verticalSpeedDelay),
	groundSpeed(groundSpeedDelay)
{
	if (ap == nullptr)
		throw std::invalid_argument("_ap");
	if (data == nullptr)
		throw std::invalid_argument("_data");
	if (timer == nullptr)
		throw std::invalid_argument("_timer");
	if (log == nullptr)
		throw std::invalid_argument("_log");

	memset(&lastSample, 0, sizeof(lastSample));
	memset(timers, 0, sizeof(timers));

	timer->setCallback(timerCallback, this);
}

FlightDirector::~FlightDirector()
{
	delete ap;
	delete data;
	delete timer;
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
	Data d;
	double Ra, Rt, dH, Ar;

	if (!data->sample(&d))
		return;
	if (_elapsedMilliseconds < 1)
		return; /* divide by zero protection. */

	Ra = rateOfTurn.pushSample((d.hdg - lastSample.hdg) * 1000 / _elapsedMilliseconds);
	verticalSpeed.pushSample((d.alt - lastSample.alt) * 1000 / _elapsedMilliseconds * 60);
	groundSpeed.pushSample(d.gs);
	lastSample = d;

	for (int i = 0; i < timerCount; ++i)
		timers[i] += _elapsedMilliseconds;
	
	updateProjectedLandingPoint();

/*	eventually code will take the updated projected landing point and
	modify targetHdg accordingly.  for now, just hold a heading.
 
	the target rate-of-turn follows an exponential curve designed to
	hit +/- 3 degrees per second at a heading error of +/- 30 degrees.
 
	the rudder angle follows a logarithmic curve with a steeper response
	when the delta between the target rate of turn and the actual rate of
	turn approaches zero.  the logarithmic curve hits +/- 0.25 degrees of
	rudder deflection at the 3 degree per second maximum rate of turn.
 */
	
	dH = fmod(fmod(targetHdg - d.hdg, 360.0) + 540.0, 360.0) - 180.0;
	Rt = min(pow(1.0472941228, min(fabs(dH), maxHdgErr)) - 1, maxRoT) * sgn(dH);
	Ar = min((log10(min(fabs(Rt - Ra), maxRoT) + .25) / 2.4082399653 + 0.24) / 1.8, maxRudder) * sgn(Rt - Ra);
	ap->setRudderDeflection((float)Ar);
	
	(*log)("H: %7.2f, dH: %7.2f, Ra: %7.2f, Rt: %7.2f, Ar: %7.2f\n",
		   d.hdg, dH, Ra, Rt, Ar);
}

void FlightDirector::updateProjectedLandingPoint()
{
	static const double R = 3440.277; //mean Earth radius in NM.
	double lat = degToRad(lastSample.lat);
	double lon = degToRad(lastSample.lon);
	double hdg = degToRad(lastSample.hdg);
	double av = verticalSpeed.average();
	double ag = groundSpeed.average();

	if (timers[projectionTimer] < projectionInterval)
		return;

	timers[projectionTimer] = 0;

/*	assume a nominal -1 ft/s if the average vertical speed is greater than -1 ft/s.  this
	both protects from division by zero and effectively assumes level flight if the glider
	is climbing.  we can recompute when the glider resumes a descent.

	clamp distance to 3,000 nm.  this keeps the projected landing point from getting silly.
	we do not actually need a perfect projected landing point.  we just need to know if the
	glider is headed in a direction that will put it in a fenced off area.

	the heading should be a ground track so that we are taking winds into account.
 */

	av = (av > -1 ? -1 : av);

	projDistance = min(lastSample.alt / (-av * 60) * ag, 3000.0);
	projLat = asin(sin(lat) * cos(projDistance / R) + cos(lat) * sin(projDistance / R) * cos(hdg));
	projLon = lon + atan2(sin(hdg) * sin(projDistance / R) * cos(lat), cos(projDistance / R) - sin(lat) * sin(projLat));

/*	convert to degrees and clamp the values. */

	projLat = max(min(radToDeg(projLat), 90.0), -90.0);
	projLon = max(min(radToDeg(projLon), 180.0), -180.0);
	
/*	for the sake of demonstration, modify the target heading with altitude. */
	
	if (lastSample.alt >= 80000)
		targetHdg = 180;
	else if (lastSample.alt < 80000 && lastSample.alt >= 60000)
		targetHdg = 90;
	else if (lastSample.alt < 60000 && lastSample.alt >= 40000)
		targetHdg = 0;
	else if (lastSample.alt < 40000 && lastSample.alt >= 20000)
		targetHdg = 270;
	else
		targetHdg = 180;
}
