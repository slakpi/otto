#ifndef ARDUINO_HPP
#define ARDUINO_HPP

#include <sys/types.h>

class Arduino
{
public:
	enum RegAddr
	{
		TEST			  = 0x01,
	};

public:
	Arduino();

public:
	~Arduino();

public:
    bool init();

	void uninit();

	void setLight(bool _on);

private:
	bool init2(u_int8_t _addr);

private:
	int fd;
};

#endif
