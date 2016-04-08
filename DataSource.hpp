#ifndef DataSource_hpp
#define DataSource_hpp

#include "Utilities.hpp"

#define DATA_POS		0x1
#define DATA_ALT		0x2
#define DATA_HDG		0x4
#define DATA_GS			0x8
#define DATA_MAGHDG		0x10
#define DATA_PITCH		0x20
#define DATA_ROLL		0x40
#define DATA_YAW		0x80

struct Data
{
	unsigned int avail;
	Loc pos;			// degrees
	double alt;			// feet
	double hdg;			// degrees (ground track)
	double gs;			// knots
	double magHdg;		// degrees (magnetic heading)
	double pitch;		// degrees
	double roll;		// degrees
	double yaw;			// degrees
};

/*******************************************************************************

 The DataSource class establishes an interface used by a FlightDirector to
 obtain current data about the aircraft.
 
 ******************************************************************************/

class DataSource
{
public:
	DataSource();
	
public:
	virtual ~DataSource();
	
public:
	virtual bool sample(Data *_data) const = 0;
};

#endif