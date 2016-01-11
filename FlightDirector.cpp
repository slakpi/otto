#include <stdexcept>
#include "FlightDirector.hpp"
#include "Utilities.hpp"

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
	projDistance(0),
	projLat(0),
	projLon(0),
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
	
	if (!data->sample(&d))
		return;
	if (_elapsedMilliseconds < 1)
		return; /* divide by zero protection. */
	
	verticalSpeed.pushSample((d.alt - lastSample.alt) * 1000 / _elapsedMilliseconds * 60);
	groundSpeed.pushSample(d.gs);
	lastSample = d;

	for (int i = 0; i < timerCount; ++i)
		timers[i] += _elapsedMilliseconds;
	
	updateProjectedLandingPoint();
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
}