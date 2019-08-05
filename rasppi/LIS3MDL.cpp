#include <cmath>
#include <unistd.h>
#include <wiringPiI2C.h>
#include "LIS3MDL.hpp"

#define LIS3MDL_SA1_HIGH_ADDRESS  0x1e
#define LIS3MDL_SA1_LOW_ADDRESS   0x1c
#define LIS3MDL_WHO_ID        0x3d

/**
 * CTRL_1_DEFAULT
 *
 *   1       10 000 1        0
 *   -       -- --- -        -
 *   TEMP_EN OM DO  FAST_ODR ST
 *
 * TEMP_EN     b1      = Temperature sensor enabled
 * OM          b10     = X & Y in high-performance mode
 * DO          xxxx    = Ignored (300 Hz when OM = b10 and FAST_ODR = b1)
 * FAST_ODR    b1      = Fast ODR disabled
 * ST          b0      = Self-test disabled
 */
#define CTRL_1_DEFAULT        0xC2

/**
 * CTRL_2_DEFAULT
 *
 *   0 00 0 0      0        0 0
 *   - -- - -      -        - -
 *   0 FS 0 REBOOT SOFT_RST 0 0
 *
 * FS      b00   = +/- 4 gauss
 */
#define CTRL_2_DEFAULT        0x00
#define MIN_MAG           -4
#define MAX_MAG            4

/**
 * CTRL_3_DEFAULT
 *
 *   0 0 0  0 0 0   00
 *   - - -  - - -   --
 *   0 0 LP 0 0 SIM MD
 *
 * LP      b0    = Low-power mode disabled
 * SIM     b0    = SPI serial interface disabled
 * MD      b00   = Continuous conversion mode
 */
#define CTRL_3_DEFAULT        0x00

/**
 * CTRL_4_DEFAULT
 *
 *   0 0 0 0 10  0   0
 *   - - - - --  -   -
 *   0 0 0 0 OMZ BLE 0
 *
 * OMZ     b10   = Z in high-performance mode
 * BLE     b0    = LSb at lower address
 */
#define CTRL_4_DEFAULT        0x04

static double makeMag(int16_t _v)
{
  return static_cast<double>(((_v + 32768) * ((MAX_MAG - MIN_MAG) / 65535.0) + MIN_MAG));
}

LIS3MDL::LIS3MDL()
: fd(-1)
{

}

LIS3MDL::~LIS3MDL()
{
  uninit();
}

bool LIS3MDL::init(Sa1State _sa1 /* = sa1_auto */)
{
  if (fd != -1)
    return true;
  if (_sa1 != sa1_low && init2(LIS3MDL_SA1_HIGH_ADDRESS))
    return true;
  if (_sa1 != sa1_high && init2(LIS3MDL_SA1_LOW_ADDRESS))
    return true;

  return false;
}

void LIS3MDL::uninit()
{
  if (fd == -1)
    return;

  close(fd);
  fd = -1;
}

void LIS3MDL::readMag(DVector &_m) const
{
  u_int8_t lx, hx, ly, hy, lz, hz;

  _m.x = _m.y = _m.z = 0;

  if (fd == -1)
    return;

  lx = wiringPiI2CReadReg8(fd, OUT_X_L);
  hx = wiringPiI2CReadReg8(fd, OUT_X_H);
  ly = wiringPiI2CReadReg8(fd, OUT_Y_L);
  hy = wiringPiI2CReadReg8(fd, OUT_Y_H);
  lz = wiringPiI2CReadReg8(fd, OUT_Z_L);
  hz = wiringPiI2CReadReg8(fd, OUT_Z_H);

  _m.x = makeMag((hx << 8) | lx);
  _m.y = makeMag((hy << 8) | ly);
  _m.z = makeMag((hz << 8) | lz);
}

int LIS3MDL::readTemp() const
{
  u_int8_t lt, ht;

  if (fd == -1)
    return 0;

  lt = wiringPiI2CReadReg8(fd, TEMP_OUT_L);
  ht = wiringPiI2CReadReg8(fd, TEMP_OUT_H);

  return static_cast<int>(((ht << 8) | lt));
}

bool LIS3MDL::init2(u_int8_t _addr)
{
  u_int16_t r;

  fd = wiringPiI2CSetup(_addr);

  if (fd == -1)
    return false;

  r = wiringPiI2CReadReg8(fd, WHO_AM_I);

  if (r == LIS3MDL_WHO_ID)
  {
    r = wiringPiI2CWriteReg8(fd, CTRL_REG1, CTRL_1_DEFAULT);
    r = wiringPiI2CWriteReg8(fd, CTRL_REG2, CTRL_2_DEFAULT);
    r = wiringPiI2CWriteReg8(fd, CTRL_REG3, CTRL_3_DEFAULT);
    r = wiringPiI2CWriteReg8(fd, CTRL_REG4, CTRL_4_DEFAULT);
    return true;
  }

  close(fd);
  fd = -1;

  return false;
}
