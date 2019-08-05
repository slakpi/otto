#ifndef RpiAutopilot_hpp
#define RpiAutopilot_hpp

#include <Autopilot.hpp>
#include "Arduino.hpp"

/**
 * RpiAutopilot implements the autopilot interface to drive the control surface
 * servos via the Arduino driver.
 */
class RpiAutopilot : public Autopilot
{
public:
  RpiAutopilot();

public:
  virtual ~RpiAutopilot();

public:
  virtual void enable();

  virtual void disable();

  virtual float getRudderDeflection() const;

  virtual void setRudderDeflection(float _deflection);

protected:
  bool enabled;
  Arduino arduino;
};

#endif
