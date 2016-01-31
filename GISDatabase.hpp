#ifndef GISDatabase_hpp
#define GISDatabase_hpp

#include "DataSource.hpp"

class GISDatabase
{
public:
	static GISDatabase db;
	
private:
	GISDatabase();
	
public:
	~GISDatabase();
	
public:
	bool getRecoveryLocation(const Loc &_ppos, double _hdg, double _maxDistance, int64_t &_id, Loc &_target);
};

#endif