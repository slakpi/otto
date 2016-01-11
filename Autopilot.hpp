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
};

#endif