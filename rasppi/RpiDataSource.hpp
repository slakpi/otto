#ifndef RpiDataSource_hpp
#define RpiDataSource_hpp

#include <pthread.h>
#include <DataSource.hpp>
#include "Vector.hpp"

struct RawData
{
  DVector g;
  DVector a;
  DVector m;
};

/**
 * RpiDataSource implements DataSource and is responsible for reading /
 * filtering data from the GPS, IMU, and magnetometer.
 */
class RpiDataSource : public DataSource
{
private:
  static void* threadProc(void *_ptr);

public:
  RpiDataSource();

public:
  virtual ~RpiDataSource();

public:
  bool start();

  bool start(const DVector &_gBias, const DVector &_mBias, const DVector &_mScale);

  bool rawSample(RawData *_rawData) const;

  void stop();

public:
  virtual bool sample(Data *_data) const;

private:
  long cancel;
  RawData curRawSample;
  Data curSample;
  pthread_t dataThread;
  pthread_mutex_t sampleLock;

  DVector gBias;  // THESE MUST NOT CHANGE WHILE THE THREAD
  DVector mBias;  // IS RUNNING. THEY ARE NOT GUARDED BY
  DVector mScale; // THE MUTEX.
};

#endif
