#ifndef GISDatabase_hpp
#define GISDatabase_hpp

#include <sys/types.h>
#include <string>
#include "DataSource.hpp"

struct RecoveryLocation
{
	int64_t id;
  char ident[9];
	Loc pos;
	double elev;
};

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

	bool getRecoveryLocation(const Loc &_ppos, double _hdg, double _maxDistance, RecoveryLocation &_loc);

private:
	void *dbhandle;
	void *cache;
};

#endif
