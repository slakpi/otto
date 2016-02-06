#include "XPlaneAutopilot.hpp"
#include "Utilities.hpp"

static const float maxRudder = 0.25f;

XPlaneAutopilot::XPlaneAutopilot()
:	fltCtrlOverrideRef(nullptr),
	rudderDeflectionRef(nullptr),
	headingBugRef(nullptr)
{
	fltCtrlOverrideRef = XPLMFindDataRef("sim/operation/override/override_control_surfaces");
	rudderDeflectionRef = XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def");
	headingBugRef = XPLMFindDataRef("sim/cockpit/autopilot/heading_mag");

	if (fltCtrlOverrideRef != nullptr)
		XPLMSetDatai(fltCtrlOverrideRef, 1);
}

XPlaneAutopilot::~XPlaneAutopilot()
{
	
}

float XPlaneAutopilot::getRudderDeflection() const
{
	if (rudderDeflectionRef == nullptr)
		return 0.0f;
	
	return XPLMGetDataf(rudderDeflectionRef) / maxRudder;
}

void XPlaneAutopilot::setRudderDeflection(float _deflection, float _hdg)
{
	if (rudderDeflectionRef == nullptr)
		return;
	
	XPLMSetDataf(rudderDeflectionRef, min(max(_deflection, -1.0f), 1.0f) * maxRudder);
	XPLMSetDataf(headingBugRef, min(max(_hdg, 0.0f), 359.0f));
}