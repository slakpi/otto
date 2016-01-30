#ifndef Autopilot_hpp
#define Autopilot_hpp

enum __ApMode
{
	apOff,
	apFD,
	apOn
};

typedef enum __ApMode ApMode;

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
	virtual ApMode apMode() const = 0;
	
	virtual float getRudderDeflection() const = 0;
	
	virtual void setRudderDeflection(float _degrees) = 0;
};

#endif