#include <cstdlib>
#include <cstring>
#include <sqlite3.h>
#include <spatialite.h>
#include "GISDatabase.hpp"

static const double nm2m = 1852.0;

GISDatabase::GISDatabase(const char *_dbPath)
: dbhandle(nullptr),
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
  sqlite3 *db;

  if (isOpen())
    closeDatabase();

  ret = sqlite3_open_v2(
   _dbPath,
   &db,
   SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_EXCLUSIVE,
   0);

  if (ret != SQLITE_OK)
  {
    if (db != nullptr)
      sqlite3_close(db);

    return false;
  }

  cache = spatialite_alloc_connection();
  spatialite_init_ex(db, cache, 0);
  dbhandle = db;

  return true;
}

void GISDatabase::closeDatabase()
{
  if (!isOpen())
    return;

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

bool GISDatabase::getRecoveryLocation(const Loc &_ppos, double _hdg, double _maxDistance, RecoveryLocation &_loc)
{
  sqlite3 *db = (sqlite3*)dbhandle;
  unsigned char *pposBlob;
  const unsigned char *targetBlob;
  int pposSize, targetSize;
  double azMin, azMax;
  gaiaGeomCollPtr g;
  sqlite3_stmt *stmt;
  int ret;

  _loc.id = -1;

  if (!isOpen())
    return false;

  /**
   * Select all recovery locations within the glide distance. Order them in
   * ascending order so that we pick the closest one. The SpatiaLite function
   * Distance() returns angular distance using SRID 4326 rather than projected
   * distance in meters. But, that's ok, we only need relative ordering.
   *
   * Limit the search to the ground track +/- 45 degrees. The flight director
   * can use this to avoid projecting a distance ring based on a tail wind and
   * then turning into the wind.
   */
  ret = sqlite3_prepare_v2(
   db,
   "SELECT pkid, ident, elev, location FROM recovery "
   "WHERE PtDistWithin(?1, location, ?2) = 1 AND "
   " Azimuth(?1, location) >= ?3 AND "
   " Azimuth(?1, location) <= ?4 "
   "ORDER BY Distance(?1, location) ASC",
   -1,
   &stmt,
   nullptr);

  if (ret != SQLITE_OK)
    return false;

  azMin = degToRad(fmod(fmod(_hdg - 45.0, 360.0) + 360.0, 360.0));
  azMax = degToRad(fmod(_hdg + 360.0, 360.0));
  gaiaMakePoint(_ppos.lon, _ppos.lat, 4326, &pposBlob, &pposSize);

  sqlite3_bind_blob(stmt, 1, pposBlob, pposSize, 0);
  sqlite3_bind_double(stmt, 2, _maxDistance * nm2m);
  sqlite3_bind_double(stmt, 3, azMin);
  sqlite3_bind_double(stmt, 4, azMax);

  ret = sqlite3_step(stmt);

  if (ret == SQLITE_ROW)
  {
    targetSize = sqlite3_column_bytes(stmt, 3);
    targetBlob = (const unsigned char*)sqlite3_column_blob(stmt, 3);
    g = gaiaFromSpatiaLiteBlobWkb(targetBlob, targetSize);
    _loc.pos.lat = g->FirstPoint->Y;
    _loc.pos.lon = g->FirstPoint->X;
    _loc.elev = sqlite3_column_double(stmt, 2);
    strncpy(_loc.ident, (const char*)sqlite3_column_text(stmt, 1), 8);
    _loc.ident[8] = 0;
    _loc.id = sqlite3_column_int64(stmt, 0);
  }

  sqlite3_finalize(stmt);
  free(pposBlob);

  return (_loc.id >= 0);
}
