#include <sqlite3.h>
#include <spatialite.h>
#include "GISDatabase.hpp"

GISDatabase::GISDatabase(const char *_dbPath)
:	dbhandle(nullptr),
	cache(nullptr)
{
	openDatabase(_dbPath);
}

GISDatabase::~GISDatabase()
{
	closeDatabase();
}

bool GISDatabase::openDatabase(const char *_dbPath)
{
	int ret;
	
	ret = sqlite3_open_v2(
	 _dbPath,
	 (sqlite3**)&dbhandle,
	 SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_EXCLUSIVE,
	 0);

	if (ret != SQLITE_OK)
	{
		if (dbhandle != nullptr)
			sqlite3_close((sqlite3*)dbhandle);
		
		return false;
	}
	
	cache = spatialite_alloc_connection();
	spatialite_init_ex((sqlite3*)dbhandle, cache, 0);
	
	return true;
}

void GISDatabase::closeDatabase()
{
	sqlite3_close((sqlite3*)dbhandle);
	spatialite_cleanup_ex(cache);
	spatialite_shutdown();
	
	dbhandle = nullptr;
	cache = nullptr;
}

bool GISDatabase::isOpen() const
{
	return (dbhandle != nullptr);
}

bool GISDatabase::getRecoveryLocation(const Loc &_ppos, double _hdg, double _maxDistance, int64_t &_id, Loc &_target)
{
	if (!isOpen())
		return false;
	
/* select ident, st_astext(location) l from recovery where PtDistWithin(location, MakePoint(44.95383, -121.46183, 4326), 138900) = 1 */
	
	return false;
}