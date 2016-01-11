#ifndef DataSource_hpp
#define DataSource_hpp

struct Data
{
	double lat; //degrees
	double lon; //degrees
	double alt; //feet
	double hdg; //degrees (ground track)
	double gs;  //knots
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