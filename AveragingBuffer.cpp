#include <stdexcept>
#include <algorithm>
#include <cstring>
#include "AveragingBuffer.hpp"
#include "Utilities.hpp"

using namespace std;

AveragingBuffer::AveragingBuffer(unsigned int _bufferLen)
: i(0),
  samples(0),
  bufferLen(_bufferLen),
  total(0)
{
  if (bufferLen < 1)
    throw std::invalid_argument("_bufferLen");

  buffer.resize(bufferLen);
  memset(buffer.data(), 0, sizeof(double) * bufferLen);
}

double AveragingBuffer::pushSample(double _sample)
{
  total -= buffer[i];
  total += _sample;
  buffer[i] = _sample;
  i = (i + 1) % bufferLen;
  samples = min(bufferLen, samples + 1);
  return total / samples;
}

double AveragingBuffer::average() const
{
  return (samples > 0 ? total / samples : 0);
}

void AveragingBuffer::reset()
{
  i = samples = 0;
  total = 0;
  memset(buffer.data(), 0, sizeof(double) * bufferLen);
}
