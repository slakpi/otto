#include "XPlaneAutopilot.hpp"
#include "Utilities.hpp"

static const float maxRudder = 0.25f;

XPlaneAutopilot::XPlaneAutopilot()
:	fltCtrlOverrideRef(nullptr),
	rudderDeflectionRef(nullptr)
{
	fltCtrlOverrideRef = XPLMFindDataRef("sim/operation/override/override_control_surfaces");
	rudderDeflectionRef = XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def");

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

void XPlaneAutopilot::setRudderDeflection(float _deflection)
{
	if (rudderDeflectionRef == nullptr)
		return;
	
	XPLMSetDataf(rudderDeflectionRef, min(max(_deflection, -1.0f), 1.0f) * maxRudder);
}