#include <Servo.h>
#include <Wire.h>

#define DEV_ADDRESS       0x08

#define LED_REG           0x01
#define SERVO_REG         0x02

Servo rudder;
unsigned char buf[2];
int recv = 0;

void setup()
{
// Setup the LED pin for debugging and turn the LED off.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
// Use the rudder servo on Pin 9 and center it.
  rudder.attach(9);
  rudder.write(90);

// Setup the I2C interface with the Raspberry Pi.
  Wire.begin(DEV_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

void loop()
{
/* Delay the main loop so that we do not run in a tight loop
 * and waste power.  The Wire library will fire an interupt
 * and break us out to do real work when necessary.
 */
  delay(1000);
}

void onI2CReceive(int _numBytes)
{

/* Loop through the received bytes and execute commands.  If
 * we receive an odd number of bytes, leave the last received
 * byte in the buffer.  onI2CRequest() will use this value to
 * determine which value to write back.
 */
  
  while (_numBytes > 0)
  {
    buf[recv++] = Wire.read(); // Read the next byte.
    
    _numBytes--;

    if (recv == 2)
    {
      switch (buf[0])
      {
      case LED_REG:
      // Turn the LED on if the second byte is non-zero.
        digitalWrite(13, buf[1] == 0 ? LOW : HIGH);
        break;
      case SERVO_REG:
      /* Clamp the rudder value to [0, 180], then move the servo.
       * This is just an extra sanity check, the Arduino driver on
       * the Raspberry Pi should already be clamping the value to
       * the real rudder range.
       */
        rudder.write(min(max(buf[1], 0), 180));
        break;
      }

      recv = 0; // Start over.
    }
  }
}

void onI2CRequest()
{
  if (recv < 1)
    return;

  switch (buf[0])
  {
  case SERVO_REG:
    Wire.write(rudder.read());
    break;
  }

  recv = 0; // Start over.
}

