#ifndef AveragingBuffer_hpp
#define AveragingBuffer_hpp

#include <vector>

class AveragingBuffer
{
public:
	AveragingBuffer(unsigned int _bufferLen);
	
public:
	double pushSample(double _sample);
	
	double average() const;
	
	void reset();
	
private:
	unsigned int i, samples, bufferLen;
	std::vector<double> buffer;
	double total;
};

#endif