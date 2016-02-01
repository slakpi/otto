#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <XPLM/XPLMPlugin.h>
#include <XPLM/XPLMPlanes.h>
#include <XPLM/XPLMUtilities.h>
#include "FlightDirector.hpp"
#include "XPlaneAutopilot.hpp"
#include "XPlaneDataSource.hpp"
#include "XPlaneTimerSource.hpp"

static FlightDirector *fd;

static void logCallback(const char *_fmt, ...)
{
	int len;
	va_list args;
	char *str;
	
	va_start(args, _fmt);
	len = vsnprintf(nullptr, 0, _fmt, args);
	
	if (len < 1)
		return;
	
	va_start(args, _fmt);
	str = new char[len + 1];
	vsnprintf(str, len + 1, _fmt, args);
	XPLMDebugString(str);
	delete [] str;
}

PLUGIN_API int XPluginStart(char *_outName, char *_outSig, char *_outDesc)
{
	char acf[256];
	char acfPath[256];
	GISDatabase *db;
	
	strncpy(_outName, "OTTO", 256);
	strncpy(_outSig, "org.or034.otto", 256);
	strncpy(_outDesc, "HALO Glider Autopilot", 256);
	
	XPLMGetNthAircraftModel(0, acf, acfPath);
	XPLMExtractFileAndPath(acfPath);
	strcat(acfPath, XPLMGetDirectorySeparator());
	strcat(acfPath, "plugins");
	strcat(acfPath, XPLMGetDirectorySeparator());
	strcat(acfPath, "otto");
	strcat(acfPath, XPLMGetDirectorySeparator());
	strcat(acfPath, "recovery.db");
	
	logCallback("OTTO attempting to open recovery database: %s\n", acfPath);
	
	db = new GISDatabase(acfPath);
	
	if (!db->isOpen())
		logCallback("OTTO failed to open recovery database.\n");
	
	fd = new FlightDirector(new XPlaneAutopilot(), new XPlaneDataSource(), new XPlaneTimerSource(), db, logCallback);

	return 1;
}

PLUGIN_API int XPluginEnable()
{
	fd->enable();
	
	return 1;
}

PLUGIN_API void XPluginDisable()
{
	fd->disable();
}

PLUGIN_API void XPluginStop()
{
	delete fd;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID _inFrom, int _inMessage, void *_inParam)
{
	
}