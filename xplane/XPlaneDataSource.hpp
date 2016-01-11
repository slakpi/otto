#ifndef XPlaneDataSource_hpp
#define XPlaneDataSource_hpp

#include <XPLM/XPLMDataAccess.h>
#include "DataSource.hpp"

/*******************************************************************************

 The XPlaneDataSource class uses the X-Plane API to obtain current data about
 the simulated aircraft.
 
 ******************************************************************************/

class XPlaneDataSource : public DataSource
{
public:
	XPlaneDataSource();
	
public:
	virtual ~XPlaneDataSource();
	
public:
	virtual bool sample(Data *_data) const;
	
private:
	XPLMDataRef lat;
	XPLMDataRef lon;
	XPLMDataRef alt;
	XPLMDataRef hdg;
	XPLMDataRef gs;
};

#endif