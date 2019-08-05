#ifndef NMEA_HPP
#define NMEA_HPP

#include <ctime>
#include <string>

/**
 * Conforms only to the values and ranges specified for the LS20031 GPS module.
 * This parser does not support the full NMEA specification.
 */
class NMEA
{
public:
  enum ParseStatus
  {
    nmeaIncomplete,
    nmeaComplete,
    nmeaCompleteInvalid,
    nmeaErrorReset
  };

  enum NMEAType
  {
    nmeaGGA,
    nmeaVTG
  };

  class NMEABase
  {
  public:
    NMEABase(NMEAType _type);

  public:
    virtual ~NMEABase();

  public:
    NMEAType getType() const;

  public:
    virtual void destroy();

  public:
    u_int8_t calculatedChecksum;
    u_int8_t messageChecksum;

  private:
    NMEAType type;
  };

  class GGA : public NMEABase
  {
  public:
    enum FixIndicator
    {
      fixInvalid,
      fixGPS,
      fixDiffGPS,
      fixDeadReckoning
    };

  public:
    GGA();

  public:
    virtual ~GGA();

  public:
    int32_t utc; // UTC in milliseconds since 00:00:00
    int32_t lat; // Latitude in ten-thousandths of a minute
    int32_t lon; // Longitude in ten-thousandths of a minute
    FixIndicator fix;
    int satellites;
    double HDOP;
    double altMSL; // Always meters
    double geoidSep; // Always meters
    double diffCorrAge;
    int diffRefID;
  };

  class VTG : public NMEABase
  {
  public:
    enum Mode
    {
      modeAutonomous,
      modeDGPS,
      modeDR,
      modeNotValid,
      modeCoarsePosition,
      modeSimulator
    };

  public:
    VTG();

  public:
    virtual ~VTG();

  public:
    double trueGTK; // Degrees true heading
    double magGTK; // Degrees magnetic heading
    double ktsGS; // Knots ground speed
    double kphGS; // km/hr ground speed
    Mode mode;
  };

private:
  enum ParseState
  {
    stateInitial,
    stateId,

    stateGGAUTC,
    stateGGALat,
    stateGGALatIndicator,
    stateGGALon,
    stateGGALonIndicator,
    stateGGAFixIndicator,
    stateGGASatellites,
    stateGGAHDOP,
    stateGGAAltitude,
    stateGGAAltUnits,
    stateGGAGeoidSep,
    stateGGAGeoidSepUnits,
    stateGGADiffCorrAge,
    stateGGADiffRefID,

    stateVTGTrueGTK,
    stateVTGTrueRef,
    stateVTGMagGTK,
    stateVTGMagRef,
    stateVTGKtsGS,
    stateVTGKtsUnit,
    stateVTGKphGS,
    stateVTGKphUnit,
    stateVTGMode,

    stateChecksum,
    stateCR,

    stateGGAInitial = stateGGAUTC,
    stateVTGInitial = stateVTGTrueGTK,
  };

public:
  NMEA();

public:
  ~NMEA();

public:
  void init();

  ParseStatus putChar(char _c, NMEABase **_sentence);

private:
  ParseState state;
  unsigned char chksum;
  std::string match;
  u_int8_t c;
  NMEABase *data;
};

#endif
