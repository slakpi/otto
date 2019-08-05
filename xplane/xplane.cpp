#include <algorithm>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#ifdef APL
#include <dlfcn.h>
#include <libgen.h>
#endif
#include <XPLM/XPLMPlugin.h>
#include <XPLM/XPLMPlanes.h>
#include <XPLM/XPLMProcessing.h>
#include <XPLM/XPLMUtilities.h>
#include "FlightDirector.hpp"
#include "Utilities.hpp"
#include "XPlaneAutopilot.hpp"
#include "XPlaneDataSource.hpp"

#define UPDATE_INTERVAL 1.0f

using namespace std;

static FlightDirector *fd;

static float flightLoopCallback(float _elapsedSinceLastCall, float _elapsedSinceLastFlightLoop, int _counter, void *_arg)
{
  fd->refresh((unsigned int)(max(_elapsedSinceLastCall, 0.0f) * 1000 + 0.5));
  
  return UPDATE_INTERVAL;
}

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
#ifdef APL
  Dl_info info;
#endif
  char path[512], *base;
  GISDatabase *db;
  
  strncpy(_outName, "OTTO", 256);
  strncpy(_outSig, "org.or034.otto", 256);
  strncpy(_outDesc, "HALO Glider Autopilot", 256);
  
#ifdef APL

  /**
   * We can't use the platform-independent XPLM functions to get the path to the
   * plugin module. They return old-style HFS paths on Macs. So, just use the
   * BSD dladdr() function on Macs. Windows and Linux can use their own system
   * APIs.
   */  
  if (dladdr((const void*)XPluginStart, &info) == 0)
    logCallback("OTTO: failed to get module path.\n");
  else
  {
    strncpy(path, info.dli_fname, 512);
    base = dirname(path);
    strncpy(path, base, 512);
    strcat(path, "/recovery.db");
  }

#endif
  
  logCallback("OTTO: attempting to open recovery database: %s\n", path);
  
  db = new GISDatabase(path);
  
  if (!db->isOpen())
    logCallback("OTTO: failed to open recovery database.\n");
  
  fd = new FlightDirector(new XPlaneAutopilot(), new XPlaneDataSource(), db, logCallback);

  XPLMRegisterFlightLoopCallback(flightLoopCallback, UPDATE_INTERVAL, fd);
  
  return 1;
}

PLUGIN_API int XPluginEnable()
{
  fd->enable();
  
  XPLMSetFlightLoopCallbackInterval(flightLoopCallback, UPDATE_INTERVAL, 0, fd);

  return 1;
}

PLUGIN_API void XPluginDisable()
{
  XPLMSetFlightLoopCallbackInterval(flightLoopCallback, 0.0f, 0, fd);
  
  fd->disable();
}

PLUGIN_API void XPluginStop()
{
  XPLMUnregisterFlightLoopCallback(flightLoopCallback, fd);
  
  delete fd;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID _inFrom, int _inMessage, void *_inParam)
{
  
}
