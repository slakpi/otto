#ifndef GISDatabase_hpp
#define GISDatabase_hpp

#include <sys/types.h>
#include "DataSource.hpp"

class GISDatabase
{
public:
	GISDatabase(const char *_dbPath);
	
public:
	~GISDatabase();
	
private:
	bool openDatabase(const char *_dbPath);
	
	void closeDatabase();
	
public:
	bool isOpen() const;
	
	bool getRecoveryLocation(const Loc &_ppos, double _hdg, double _maxDistance, int64_t &_id, Loc &_target);
	
private:
	void *dbhandle;
	void *cache;
};

#endif