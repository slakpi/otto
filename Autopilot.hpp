#ifndef Autopilot_hpp
#define Autopilot_hpp

/*******************************************************************************
 
 The Autopilot class establishes an interface used by a FlightDirector to
 carry out instructions.
 
 ******************************************************************************/

class Autopilot
{
public:
	Autopilot();
	
public:
	virtual ~Autopilot();
	
public:
	virtual float getRudderDeflection() const = 0;
	
	virtual void setRudderDeflection(float _degrees) = 0;
};

#endif