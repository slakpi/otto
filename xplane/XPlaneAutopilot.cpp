#include "XPlaneAutopilot.hpp"

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
	
	return XPLMGetDataf(rudderDeflectionRef);
}

void XPlaneAutopilot::setRudderDeflection(float _degrees)
{
	if (rudderDeflectionRef == nullptr)
		return;
	
	XPLMSetDataf(rudderDeflectionRef, _degrees);
}