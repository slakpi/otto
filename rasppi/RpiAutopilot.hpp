#ifndef RpiAutopilot_hpp
#define RpiAutopilot_hpp

#include <Autopilot.hpp>

/*******************************************************************************
 
 
 ******************************************************************************/

class RpiAutopilot : public Autopilot
{
public:
	RpiAutopilot();
	
public:
	virtual ~RpiAutopilot();
	
public:
	virtual float getRudderDeflection() const;
	
	virtual void setRudderDeflection(float _deflection);
};

#endif
