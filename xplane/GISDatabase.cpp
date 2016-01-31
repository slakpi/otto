#include <sqlite3.h>
#include <spatialite.h>
#include "GISDatabase.hpp"

static const Loc recoveryPoints[] = {
	{	45.4315, -122.9425	}, // Twin Oaks
	{	45.3093, -122.3218	}, // Valley View
	{	44.5432, -122.9315	}, // Lebanon
	{	44.6735, -121.16	}, // Madras
};

static const int recoveryPointCount = COUNTOF(recoveryPoints);

GISDatabase GISDatabase::db;

GISDatabase::GISDatabase()
{
	
}

GISDatabase::~GISDatabase()
{
	
}

bool GISDatabase::getRecoveryLocation(const Loc &_ppos, double _hdg, double _maxDistance, int64_t &_id, Loc &_target)
{
	return false;
}