#include <sys/types.h>
#include <sys/stat.h>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <cerrno>
#include <unistd.h>
#include <syslog.h>
#include <cstring>
#include <csignal>
#include <cstdarg>
#include <wiringPi.h>
#include "LSM6.hpp"
#include "LIS3MDL.hpp"

static int running = 1;

static void signalHandler(int _signum)
{
	switch (_signum)
	{
	case SIGINT:
	case SIGTERM:
		running = 0;
		break;
	}
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
	syslog(LOG_INFO, str);
	delete [] str;
}

static void doTasks()
{
	LSM6 lsm6;
	LIS3MDL lis3mdl;
	Vector<double> a, g, m;

	lsm6.init();
	lsm6.readAccel(a);
	lsm6.readGyro(g);

	lis3mdl.init();
	lis3mdl.readMag(m);
}

#ifndef OTTO_DAEMON
int main(int _argc, char* _argv[])
{
	doTasks();

	return 0;
}
#else
int main(int _argc, char* _argv[])
{
	pid_t pid, sid;
	int loop = 0;

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	openlog("otto", LOG_NDELAY | LOG_PID, LOG_USER);
	syslog(LOG_INFO, "OTTO daemon starting...");

	pid = fork();
	if (pid < 0)	return -1;
	if (pid > 0)	return 0;

	umask(0);

	sid = setsid();
	if (sid < 0)
		return -1;

	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);

	while (running != 0)
	{
		doTasks();
	}

	closelog();

	return 0;
}
#endif
