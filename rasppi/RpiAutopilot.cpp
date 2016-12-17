#include "RpiAutopilot.hpp"
#include "Utilities.hpp"

RpiAutopilot::RpiAutopilot()
:	enabled(false)
{
	arduino.init();
}

RpiAutopilot::~RpiAutopilot()
{

}

void RpiAutopilot::enable()
{
	enabled = true;
}

void RpiAutopilot::disable()
{
	enabled = false;
}

float RpiAutopilot::getRudderDeflection() const
{
	return arduino.getServoPos();
}

void RpiAutopilot::setRudderDeflection(float _deflection)
{
	if (!enabled)
		return;

	arduino.setServoPos(_deflection);
}
