#ifndef LS20031_HPP
#define LS20031_HPP

#include <pthread.h>

class LS20031
{
public:
	struct GpsSample
	{
		double lat;
		double lon;
		double alt;
		double gs;
		double hdg;
	};

private:
	static void* threadProc(void *_param);

public:
	LS20031();

public:
	~LS20031();

public:
	bool init(const char *_dev);

	bool sample(GpsSample &_sample, bool block = true);

	void uninit();

private:
	int fd, run;
	pthread_t td;
	pthread_mutex_t sampleLock;
	GpsSample curData;
};

#endif
