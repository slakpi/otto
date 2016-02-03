#ifndef RpiDataSource_hpp
#define RpiDataSource_hpp

#include <DataSource.hpp>

/*******************************************************************************

 ******************************************************************************/

class RpiDataSource : public DataSource
{
public:
	RpiDataSource();

public:
	virtual ~RpiDataSource();

public:
	virtual bool sample(Data *_data) const;
};

#endif
