#ifndef Autopilot_hpp
#define Autopilot_hpp

/*******************************************************************************
 
 The Autopilot class establishes an interface used by a FlightDirector to
 carry out instructions.
 
 Rudder deflection must be in the range [-1, 1].  The Autopilot subclasses will
 translate that into platform-specific units.  -1 is full left deflection, and
 1 is full right deflection.
 
 ******************************************************************************/

class Autopilot
{
public:
	Autopilot();
	
public:
	virtual ~Autopilot();
	
public:
	virtual void enable() = 0;
	
	virtual void disable() = 0;
	
	virtual float getRudderDeflection() const = 0;
	
	virtual void setRudderDeflection(float _deflection) = 0;
};

#endif