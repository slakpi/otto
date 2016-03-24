#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "LS20031.hpp"

void* LS20031::threadProc(void *_param)
{
	pthread_exit(NULL);
}

LS20031::LS20031()
:	fd(-1),
	run(0)
{

}

LS20031::~LS20031()
{
	uninit();
}

bool LS20031::init(const char *_dev)
{
	int ret;
	pthread_attr_t attr;

	if (fd != -1)
		return false;

	fd = serialOpen(_dev, 57600);

	if (fd == -1)
		return false;

	pthread_mutex_init(&sampleLock, NULL);
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	ret = pthread_create(&td, &attr, threadProc, static_cast<void*>(this));
	pthread_attr_destroy(&attr);

	if (ret != 0)
	{
		pthread_mutex_destroy(&sampleLock);
		close(fd);
		fd = -1;
		return false;
	}

	return true;
}

bool LS20031::sample(GpsSample &_sample, bool _block /* = true */)
{
	return false;
}

void LS20031::uninit()
{
	if (fd == -1)
		return;
}
