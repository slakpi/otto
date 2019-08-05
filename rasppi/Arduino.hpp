#ifndef ARDUINO_HPP
#define ARDUINO_HPP

#include <sys/types.h>

class Arduino
{
public:
  enum RegAddr
  {
    LED_REG   = 0x01,
    SERVO_REG = 0x02
  };

public:
  Arduino();

public:
  ~Arduino();

public:
    bool init();

  void uninit();

  void setLight(bool _on);

  float getServoPos() const;

  void setServoPos(float _Prel);

private:
  int fd;
};

#endif
