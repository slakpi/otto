#ifndef LS20031_HPP
#define LS20031_HPP

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

public:
	LS20031();

public:
	~LS20031();

public:
	bool init(const char *_dev);

	bool sample(GpsSample &_sample);

	void uninit();

private:
	int fd;
};

#endif
