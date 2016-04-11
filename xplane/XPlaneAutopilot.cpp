#include <algorithm>
#include "XPlaneAutopilot.hpp"
#include "Utilities.hpp"

using namespace std;

static const float maxRudder = 0.5f;

XPlaneAutopilot::XPlaneAutopilot()
:	fltCtrlOverrideRef(nullptr),
	rudderDeflectionRef(nullptr)
{
	fltCtrlOverrideRef = XPLMFindDataRef("sim/operation/override/override_control_surfaces");
	rudderDeflectionRef = XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def");
	enable();
}

XPlaneAutopilot::~XPlaneAutopilot()
{

}

void XPlaneAutopilot::enable()
{
	if (fltCtrlOverrideRef != nullptr)
		XPLMSetDatai(fltCtrlOverrideRef, 1);
}

void XPlaneAutopilot::disable()
{
	if (fltCtrlOverrideRef != nullptr)
		XPLMSetDatai(fltCtrlOverrideRef, 0);
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
