#include "XPlaneAutopilot.hpp"

XPlaneAutopilot::XPlaneAutopilot()
:	apModeRef(nullptr),
	fltCtrlOverrideRef(nullptr),
	rudderDeflectionRef(nullptr)
{
	apModeRef = XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode");
	fltCtrlOverrideRef = XPLMFindDataRef("sim/operation/override/override_control_surfaces");
	rudderDeflectionRef = XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def");
}

XPlaneAutopilot::~XPlaneAutopilot()
{
	
}

ApMode XPlaneAutopilot::apMode() const
{
	ApMode mode = apOn;

#if 0
	int m;
	
	if (apModeRef == nullptr)
		return apOff;
	
	m = XPLMGetDatai(apModeRef);
	
	switch (m)
	{
		default:
		case 0:		mode = apOff;
		case 1:		mode = apFD;
		case 2:		mode = apOn;
	}
#endif
	
	if (fltCtrlOverrideRef != nullptr)
		XPLMSetDatai(fltCtrlOverrideRef, mode == apOn);
	
	return mode;
}

float XPlaneAutopilot::getRudderDeflection() const
{
	if (rudderDeflectionRef == nullptr)
		return 0.0f;
	
	return XPLMGetDataf(rudderDeflectionRef);
}

void XPlaneAutopilot::setRudderDeflection(float _degrees)
{
	if (apMode() != apOn)
		return;
	if (rudderDeflectionRef == nullptr)
		return;
	
	XPLMSetDataf(rudderDeflectionRef, _degrees);
}