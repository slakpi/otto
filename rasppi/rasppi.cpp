#include <unistd.h>
#include <cstdlib>
#include <csignal>
#include <cfloat>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <ncurses.h>
#include <wiringPi.h>
#include <AveragingBuffer.hpp>
#include "RpiDataSource.hpp"

using namespace std;

static int running = 1;

static const char *title = "PREFLIGHT CHECKLIST";

static const char* checklist[] = {
	"[%c] Calibrate magnetometer",
	"[%c] Calibrate gyroscope",
	"[ ] Monitor data"
};

static int itemComplete[] = {
	0,
	0,
	0
};

static const int items = sizeof(checklist) / sizeof(char*);

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

#if 0
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
#endif

static int initCurses(WINDOW **w)
{
	int rows, cols;

	initscr();
	getmaxyx(stdscr, rows, cols);
	*w = newwin(rows - 1, cols, 1, 0);
	box(*w, 0, 0);
	noecho();
	keypad(*w, TRUE);
	curs_set(0);

	return 0;
}

static int menu(WINDOW *w)
{
	int i, ch, c = 0, rows, cols;

	if (itemComplete[0]) ++c;
	if (itemComplete[1]) ++c;

	getmaxyx(w, rows, cols);
	mvwprintw(w, 0, (cols - strlen(title)) / 2, "%s", title);

	for (i = 0; i < items; ++i)
	{
		if (i == c)
			wattron(w, A_STANDOUT);
		else
			wattroff(w, A_STANDOUT);

		mvwprintw(w, i + 1, 2, checklist[i], (itemComplete[i] != 0 ? 'X' : ' '));
	}

	wrefresh(w);

	while (true)
	{
		ch = wgetch(w);
		mvwprintw(w, c + 1, 2, checklist[c], (itemComplete[c] != 0 ? 'X' : ' '));

		switch (ch)
		{
		case 'q':
			return -1;
		case KEY_UP:
			c = max(c - 1, 0);
			break;
		case KEY_DOWN:
			c = min(c + 1, items - 1);
			break;
		case 0xa:
		case KEY_ENTER:
			return c;
		}

		wattron(w, A_STANDOUT);
		mvwprintw(w, c + 1, 2, checklist[c], (itemComplete[c] != 0 ? 'X' : ' '));
		wattroff(w, A_STANDOUT);
	}

	return -1;
}

static int calMag(RpiDataSource *_rds, DVector *_mBias, DVector *_mScale)
{
	RawData rawSample;
	DVector mMax = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
	DVector mMin = {DBL_MAX, DBL_MAX, DBL_MAX};
	double avg;
	int i;

	if (!_rds->start())
		return 0;

	for (i = 0; i < 2400; ++i)
	{
		_rds->rawSample(&rawSample);

		mMax.x = max(mMax.x, rawSample.m.x);
		mMax.y = max(mMax.y, rawSample.m.y);
		mMax.z = max(mMax.z, rawSample.m.z);

		mMin.x = min(mMin.x, rawSample.m.x);
		mMin.y = min(mMin.y, rawSample.m.y);
		mMin.z = min(mMin.z, rawSample.m.z);

		usleep(25000);
	}

	_rds->stop();

	_mBias->x = (mMax.x + mMin.x) / 2.0;
	_mBias->y = (mMax.y + mMin.y) / 2.0;
	_mBias->z = (mMax.z + mMin.z) / 2.0;

	_mScale->x = (mMax.x - mMin.x) / 2.0;
	_mScale->y = (mMax.y - mMin.y) / 2.0;
	_mScale->z = (mMax.z - mMin.z) / 2.0;
	avg = (_mScale->x + _mScale->y + _mScale->z) / 3.0;

	_mScale->x = avg / _mScale->x;
	_mScale->y = avg / _mScale->y;
	_mScale->z = avg / _mScale->z;

	return 1;
}

static int calGyro(RpiDataSource *_rds, DVector *_gBias)
{
	RawData rawSample;
	AveragingBuffer gx(400), gy(400), gz(400);
	int i;

	if (!_rds->start())
		return 0;

	for (i = 0; i < 2400; ++i)
	{
		_rds->rawSample(&rawSample);

		_gBias->x = gx.pushSample(rawSample.g.x);
		_gBias->y = gy.pushSample(rawSample.g.y);
		_gBias->z = gz.pushSample(rawSample.g.z);

		usleep(25000);
	}

	_rds->stop();

	return 1;
}

static void killCurses(WINDOW *w)
{
	delwin(w);
	endwin();
}

int main(int _argc, char* _argv[])
{
	RpiDataSource rds;
	Data sample;
	DVector mBias, mScale, gBias;
	WINDOW *w;
	int go = 0;

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	wiringPiSetup();
	initCurses(&w);

	while (go == 0)
	{
		switch (menu(w))
		{
		case 0:
			itemComplete[0] = calMag(&rds, &mBias, &mScale);

			if (itemComplete[0] == 0)
			{
				killCurses(w);
				cerr << "Failed to calibrate magnetometer.\n";
				return -1;
			}

			break;
		case 1:
			itemComplete[1] = calGyro(&rds, &gBias);

			if (itemComplete[1] == 0)
			{
				killCurses(w);
				cerr << "Failed to calibrate gyroscope.\n";
				return -1;
			}

			break;
		case 2:
			if (itemComplete[0] == 0 || itemComplete[1] == 0)
				break;

			go = 1;
			break;
		default:
			go = -1;
			break;
		}
	}

	if (go < 0)
	{
		killCurses(w);
		return 0;
	}

	if (!rds.start(gBias, mBias, mScale))
	{
		killCurses(w);
		cerr << "Failed to start data source.\n";
		return -1;
	}

	delwin(w);
	clear();

	while (running != 0)
	{
		rds.sample(&sample);

		if (sample.yaw < 0)
			sample.yaw += 360;

		mvprintw(0, 0, "\rYaw: %6.1f  Pitch: %6.1f  Roll: %6.1f",
			sample.yaw,
			sample.pitch,
			sample.roll);
		refresh();
		usleep(500000);
	}

	rds.stop();
	clear();
	endwin();

	return 0;
}
