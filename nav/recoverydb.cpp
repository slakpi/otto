#include <iostream>
#include <string>
#include <sstream>
#include <ctime>
#include <sqlite3.h>
#include <spatialite.h>

struct Coord
{
	int s;
	int deg;
	int min;
	double sec;
};

struct LatLon
{
	Coord lat;
	Coord lon;
};

enum Field
{
	aptInvalidField = -1,
	aptIdent = 0,
	aptLatitude = 1,
	aptLongitude = 2
};

static int _parseCoord(const char *_str, Coord *_coord)
{
	char *stop = nullptr;

	_coord->s = 1;
	_coord->deg = (int)strtol(_str, &stop, 10), stop++;
	_coord->min = (int)strtol(stop, &stop, 10), stop++;
	_coord->sec = strtod(stop, &stop);

	if (*stop == 'S' || *stop == 'W')
		_coord->s = -1;

	return 0;
}

static int _readRecoveryLocations(const char *_path, sqlite3 *_db)
{
	FILE *data;
	char buf[32768];
	size_t i, j, bytes;
	std::stringstream sql;
	std::string ident;
	std::string lat;
	std::string lon;
	LatLon latLon;
	int field = 0, isEOL, isSep, ret, ok = 0;
	int isFirstLine = 1;
	double latdd, londd;
	gaiaGeomCollPtr ptGeo;
	unsigned char *ptBlob;
	int ptSize;
	sqlite3_stmt *stmt;

	data = fopen(_path, "r");

	if (!data)
		return -1;

	try
	{
		ret = sqlite3_prepare(
		 _db,
		 "INSERT INTO Recovery(ident, location) VALUES(?, ?)",
		 -1,
		 &stmt,
		 0);

		if (ret != SQLITE_OK)
			throw ret;

		for(;;)
		{
			bytes = fread(buf, sizeof(char), 32768, data);

			if (bytes == 0)
				break;

			for(i = 0, j = 0; i < bytes; i++)
			{
				isEOL = (buf[i] == '\n');
				isSep = (isEOL || buf[i] == ',');

				if (isFirstLine)
				{
					if (isEOL)
					{
						isFirstLine = 0;
						j = i + 1;
					}

					continue;
				}

				if (isSep || i == bytes - 1)
				{
					switch (field)
					{
					case aptIdent:
						ident.append(&buf[j], i - j);
						break;
					case aptLatitude:
						lat.append(&buf[j], i - j);
						break;
					case aptLongitude:
						lon.append(&buf[j], i - j);
						break;
					}

					j = i + 1;

					if (isSep)
						field++;
				}

				if (isEOL)
				{
					_parseCoord(lat.c_str(), &latLon.lat);
					latdd = latLon.lat.deg;
					latdd += latLon.lat.min / 60.0;
					latdd += latLon.lat.sec / 3600.0;
					latdd *= latLon.lat.s;

					_parseCoord(lon.c_str(), &latLon.lon);
					londd = latLon.lon.deg;
					londd += latLon.lon.min / 60.0;
					londd += latLon.lon.sec / 3600.0;
					londd *= latLon.lon.s;

					ptGeo = gaiaAllocGeomColl();
					ptGeo->Srid = 4326;
					gaiaAddPointToGeomColl(ptGeo, latdd, londd);
					gaiaToSpatiaLiteBlobWkb(ptGeo, &ptBlob, &ptSize);
					gaiaFreeGeomColl(ptGeo);

					sqlite3_reset(stmt);
					sqlite3_clear_bindings(stmt);

					if (ident.size() > 0)
						sqlite3_bind_text(stmt, 1, &(*ident.begin()), -1, 0);

					sqlite3_bind_blob(stmt, 2, ptBlob, ptSize, free);

					ret = sqlite3_step(stmt);

					if (ret != SQLITE_DONE && ret != SQLITE_ROW)
						throw ret;

					ident.clear();
					lat.clear();
					lon.clear();
					field = 0;
					j = i + 1;
				}
			}
		}

		ok = 1;
	}
	catch (int)
	{
	}

	if (stmt)
		sqlite3_finalize(stmt);
	if (data)
		fclose(data);
	if (ok)
		return 0;

	return -1;
}

int main(int _argc, char* _argv[])
{
	int ret, ok = 0;
	sqlite3 *db = 0;
	void *cache = 0;

	if (_argc < 3)
		return -1;

	try
	{
		ret = sqlite3_open_v2(
		 _argv[1],
		 &db,
		 SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_EXCLUSIVE,
		 0);

		if (ret != SQLITE_OK)
			throw ret;

		cache = spatialite_alloc_connection();
		spatialite_init_ex(db, cache, 0);

		ret = sqlite3_exec(
		 db,
		 "SELECT InitSpatialMetadata(1)",
		 0,
		 0,
		 0);

		if (ret != SQLITE_OK)
			throw ret;

		ret = sqlite3_exec(
		 db,
		 "CREATE TABLE Recovery( "
		 " pkid INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT, "
		 " ident TEXT NOT NULL);",
		 0,
		 0,
		 0);

		if (ret != SQLITE_OK)
			throw ret;

		ret = sqlite3_exec(
		 db,
		 "SELECT AddGeometryColumn('Recovery', 'location', 4326, 'POINT', 'XY', 0)",
		 0,
		 0,
		 0);

		if (ret != SQLITE_OK)
			throw ret;

		ret = sqlite3_exec(
		 db,
		 "SELECT CreateMbrCache('Recovery', 'location')",
		 0,
		 0,
		 0);

		if (ret != SQLITE_OK)
			throw ret;

		if (_readRecoveryLocations(_argv[2], db) == 0)
			ok = 1;
	}
	catch (int)
	{
	}

	if (db)
	{
		sqlite3_close(db);
		spatialite_cleanup_ex(cache);
		spatialite_shutdown();
	}
	if (ok)
		return 0;

	return -1;
}
