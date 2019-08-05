#include "NMEA.hpp"

NMEA::NMEABase::NMEABase(NMEAType _type)
: calculatedChecksum(0),
  messageChecksum(0),
  type(_type)
{

}

NMEA::NMEABase::~NMEABase()
{

}

NMEA::NMEAType NMEA::NMEABase::getType() const
{
  return type;
}

void NMEA::NMEABase::destroy()
{
  delete this;
}

NMEA::GGA::GGA()
: NMEABase(nmeaGGA),
  utc(0),
  lat(0),
  lon(0),
  fix(fixInvalid),
  satellites(0),
  HDOP(0),
  altMSL(0),
  geoidSep(0),
  diffCorrAge(0),
  diffRefID(0)
{

}

NMEA::GGA::~GGA()
{

}

NMEA::VTG::VTG()
: NMEABase(nmeaVTG),
  trueGTK(0),
  magGTK(0),
  ktsGS(0),
  kphGS(0),
  mode(modeNotValid)
{

}

NMEA::VTG::~VTG()
{

}

NMEA::NMEA()
: state(stateInitial),
  chksum(0),
  c(0),
  data(NULL)
{

}

NMEA::~NMEA()
{

}

void NMEA::init()
{
  state = stateInitial;
  chksum = 0;
  c = 0;

  if (data != NULL)
    delete data;

  data = NULL;
}

#define _GGA    (static_cast<GGA*>(data))
#define _VTG    (static_cast<VTG*>(data))
#define _REJECT   0x0
#define _ALLOW    0x1
#define _ALPHA    0x3
#define _NUM    0x5
#define REJECT(_c)  (map[static_cast<unsigned char>(_c)] == _REJECT)
#define ALLOW(_c) (map[static_cast<unsigned char>(_c)] & _ALLOW)
#define ALPHA(_c) (map[static_cast<unsigned char>(_c)] & _ALPHA)
#define NUM(_c)   (map[static_cast<unsigned char>(_c)] & _NUM)
#define CLR()   (match.clear(), c = 0)
#define ERR()   return init(), nmeaErrorReset

static const unsigned char map[] =
{
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  1,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  1,  0,  1,  1,  1,  0,
  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  0,  0,  0,  0,  0,  0,
  0,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,
  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};

static bool parseTimeString(const char *_str, size_t _len, int32_t *_ms)
{
  /**
   * Assumes a format of HHMMSS.SSS where H, M, and S are members of the set
   * [0-9]. The value must not exceed 23:59:59.999. An int32_t can safely
   * accomodate 100:00:00.000, so there is no threat of aliasing an invalid
   * value to a valid value.
   */
  *_ms = 0;

  if (_len != 10)
    return false;

  *_ms += (_str[0] - '0') * 10 * 3600 * 1000;
  *_ms += (_str[1] - '0') * 3600 * 1000;
  *_ms += (_str[2] - '0') * 10 * 60 * 1000;
  *_ms += (_str[3] - '0') * 60 * 1000;
  *_ms += (_str[4] - '0') * 10 * 1000;
  *_ms += (_str[5] - '0') * 1000;
  *_ms += (_str[7] - '0') * 100;
  *_ms += (_str[8] - '0') * 10;
  *_ms += (_str[9] - '0');

  if (*_ms > 24 * 3600 * 1000 - 1)
  {
    *_ms = 0;
    return false;
  }

  return true;
}

static bool parseLat(const char *_str, size_t _len, int32_t *_lat)
{
  /**
   * Assumes a format of DDMM.MMMM where D and M are members of the set [0-9].
   *
   * Express the value as an integer representing ten-thousandths of a minute.
   * This preserves fidelity of the value until we need to convert it for use
   * elsewhere.
   *
   * The value must not exceed 90 degrees. An int32_t can acommodate 1000
   * degrees with a sign bit, so there is no threat of aliasing an invalid value
   * to a valid value.
   */
  *_lat = 0;

  if (_len != 9)
    return false;

  *_lat += (_str[0] - '0') * 10 * 60 * 10000;
  *_lat += (_str[1] - '0') * 60 * 10000;
  *_lat += (_str[2] - '0') * 10 * 10000;
  *_lat += (_str[3] - '0') * 10000;
  *_lat += (_str[5] - '0') * 1000;
  *_lat += (_str[6] - '0') * 100;
  *_lat += (_str[7] - '0') * 10;
  *_lat += (_str[8] - '0');

  if (*_lat > 90 * 60 * 10000)
  {
    *_lat = 0;
    return false;
  }

  return true;
}

static bool parseLon(const char *_str, size_t _len, int32_t *_lon)
{
  /**
   * Assumes a format of DDDMM.MMMM where D and M are members of the set [0-9].
   *
   * Express the value as an integer representing ten-thousandths of a minute.
   * This preserves fidelity of the value until we need to convert it for use
   * elsewhere.
   *
   * The value must not exceed 179 - 0.0001 degrees. An int32_t can acommodate
   * 1000 degrees with a sign bit, so there is no threat of aliasing an invalid
   * value to a valid value.
   */
  *_lon = 0;

  if (_len != 10)
    return false;

  *_lon += (_str[0] - '0') * 100 * 60 * 10000;
  *_lon += (_str[1] - '0') * 10 * 60 * 10000;
  *_lon += (_str[2] - '0') * 60 * 10000;
  *_lon += (_str[3] - '0') * 10 * 10000;
  *_lon += (_str[4] - '0') * 10000;
  *_lon += (_str[6] - '0') * 1000;
  *_lon += (_str[7] - '0') * 100;
  *_lon += (_str[8] - '0') * 10;
  *_lon += (_str[9] - '0');

  if (*_lon > 180 * 60 * 10000 - 1)
  {
    *_lon = 0;
    return false;
  }

  return true;
}

NMEA::ParseStatus NMEA::putChar(char _c, NMEABase **_sentence)
{
  *_sentence = NULL;

  if (REJECT(_c))
    ERR();

  c++;
  chksum ^= _c;

  switch (state)
  {
  case stateInitial:
    if (_c != '$')
      return nmeaIncomplete;

    state = stateId;
    chksum = 0;
    CLR();
    break;

  case stateId:
    if (_c == ',')
    {
      if (match == "GPGGA")
      {
        data = new GGA();
        state = stateGGAInitial;
      }
      else if (match == "GPVTG")
      {
        data = new VTG();
        state = stateVTGInitial;
      }
      else
      {
        init();
        break;
      }

      CLR();
      break;
    }

    if (!ALPHA(_c) || c > 5)
      ERR();

    match.push_back(_c);
    break;

  case stateGGAUTC:
    if (_c == ',')
    {
      if (!parseTimeString(match.c_str(), match.size(), &_GGA->utc))
        ERR();

      state = stateGGALat;
      CLR();
      break;
    }

    if ((!NUM(_c) && _c != '.') || c > 10)
      ERR();

    match.push_back(_c);
    break;

  case stateGGALat:
    if (_c == ',')
    {
      if (!parseLat(match.c_str(), match.size(), &_GGA->lat))
        ERR();

      state = stateGGALatIndicator;
      CLR();
      break;
    }

    if ((!NUM(_c) && _c != '.') || c > 9)
      ERR();

    match.push_back(_c);
    break;

  case stateGGALatIndicator:
    switch (_c)
    {
    case ',':
      state = stateGGALon;
      CLR();
      break;
    case 'N':
      break;
    case 'S':
      _GGA->lat = -_GGA->lat;
      break;
    default:
      ERR();
    }

    if (c > 1)
      ERR();

    break;

  case stateGGALon:
    if (_c == ',')
    {
      if (!parseLon(match.c_str(), match.size(), &_GGA->lon))
        ERR();

      state = stateGGALonIndicator;
      CLR();
      break;
    }

    if ((!NUM(_c) && _c != '.') || c > 10)
      ERR();

    match.push_back(_c);
    break;

  case stateGGALonIndicator:
    switch (_c)
    {
    case ',':
      state = stateGGAFixIndicator;
      CLR();
      break;
    case 'E':
      break;
    case 'W':
      _GGA->lon = -_GGA->lon;
      break;
    default:
      ERR();
    }

    if (c > 1)
      ERR();

    break;

  case stateGGAFixIndicator:
    if (_c == ',')
    {
      state = stateGGASatellites;
      CLR();
      break;
    }

    if (c > 1)
      ERR();

    switch (_c)
    {
    case '0':
      _GGA->fix = GGA::fixInvalid;
      break;
    case '1':
      _GGA->fix = GGA::fixGPS;
      break;
    case '2':
      _GGA->fix = GGA::fixDiffGPS;
      break;
    case '6':
      _GGA->fix = GGA::fixDeadReckoning;
      break;
    default:
      ERR();
    }

    break;

  case stateGGASatellites:
    if (_c == ',')
    {
      _GGA->satellites = static_cast<int>(strtol(match.c_str(), NULL, 10));
      state = stateGGAHDOP;
      CLR();
      break;
    }

    if (!NUM(_c) || c > 2)
      ERR();

    match.push_back(_c);
    break;

  case stateGGAHDOP:
    if (_c == ',')
    {
      _GGA->HDOP = strtod(match.c_str(), NULL);
      state = stateGGAAltitude;
      CLR();
      break;
    }

    if (!NUM(_c) && _c != '.' && _c != '-')
      ERR();

    match.push_back(_c);
    break;

  case stateGGAAltitude:
    if (_c == ',')
    {
      _GGA->altMSL = strtod(match.c_str(), NULL);
      state = stateGGAAltUnits;
      CLR();
      break;
    }

    if (!NUM(_c) && _c != '.' && _c != '-')
      ERR();

    match.push_back(_c);
    break;

  case stateGGAAltUnits:
    if (_c == ',')
    {
      state = stateGGAGeoidSep;
      CLR();
      break;
    }

    if (_c != 'M' || c > 1)
      ERR();

    break;

  case stateGGAGeoidSep:
    if (_c == ',')
    {
      _GGA->geoidSep = strtod(match.c_str(), NULL);
      state = stateGGAGeoidSepUnits;
      CLR();
      break;
    }

    if (!NUM(_c) && _c != '.' && _c != '-')
      ERR();

    match.push_back(_c);
    break;

  case stateGGAGeoidSepUnits:
    if (_c == ',')
    {
      state = stateGGADiffCorrAge;
      CLR();
      break;
    }

    if (_c != 'M' || c > 1)
      ERR();

    break;

  case stateGGADiffCorrAge:
    if (_c == ',')
    {
      _GGA->diffCorrAge = strtod(match.c_str(), NULL);
      state = stateGGADiffRefID;
      CLR();
      break;
    }

    if (!NUM(_c) && _c != '.' && _c != '-')
      ERR();

    match.push_back(_c);
    break;

  case stateGGADiffRefID:
    if (_c == '*')
    {
      _GGA->diffRefID = static_cast<int>(strtol(match.c_str(), NULL, 10));
      state = stateChecksum;
      chksum ^= _c; // Undo checksum XOR.
      CLR();
      break;
    }

    if (!NUM(_c) || c > 4)
      ERR();

    match.push_back(_c);
    break;

  case stateVTGTrueGTK:
    if (_c == ',')
    {
      _VTG->trueGTK = strtod(match.c_str(), NULL);
      state = stateVTGTrueRef;
      CLR();
      break;
    }

    if (!NUM(_c) && _c != '.')
      ERR();

    match.push_back(_c);
    break;

  case stateVTGTrueRef:
    switch (_c)
    {
    case ',':
      state = stateVTGMagGTK;
      CLR();
      break;
    case 'T':
      break;
    default:
      ERR();
    }

    if (c > 1)
      ERR();

    break;

  case stateVTGMagGTK:
    if (_c == ',')
    {
      _VTG->magGTK = strtod(match.c_str(), NULL);
      state = stateVTGMagRef;
      CLR();
      break;
    }

    if (!NUM(_c) && _c != '.')
      ERR();

    match.push_back(_c);
    break;

  case stateVTGMagRef:
    switch (_c)
    {
    case ',':
      state = stateVTGKtsGS;
      CLR();
      break;
    case 'M':
      break;
    default:
      ERR();
    }

    if (c > 1)
      ERR();

    break;

  case stateVTGKtsGS:
    if (_c == ',')
    {
      _VTG->ktsGS = strtod(match.c_str(), NULL);
      state = stateVTGKtsUnit;
      CLR();
      break;
    }

    if (!NUM(_c) && _c != '.')
      ERR();

    match.push_back(_c);
    break;

  case stateVTGKtsUnit:
    switch (_c)
    {
    case ',':
      state = stateVTGKphGS;
      CLR();
      break;
    case 'N':
      break;
    default:
      ERR();
    }

    if (c > 1)
      ERR();

    break;

  case stateVTGKphGS:
    if (_c == ',')
    {
      _VTG->kphGS = strtod(match.c_str(), NULL);
      state = stateVTGKphUnit;
      CLR();
      break;
    }

    if (!NUM(_c) && _c != '.')
      ERR();

    match.push_back(_c);
    break;

  case stateVTGKphUnit:
    switch (_c)
    {
    case ',':
      state = stateVTGMode;
      CLR();
      break;
    case 'K':
      break;
    default:
      ERR();
    }

    if (c > 1)
      ERR();

    break;

  case stateVTGMode:
    if (_c == '*')
    {
      chksum ^= _c; // Undo checksum XOR.
      state = stateChecksum;
      CLR();
      break;
    }

    if (c > 1)
      ERR();

    switch (_c)
    {
    case 'A':
      _VTG->mode = VTG::modeAutonomous;
      break;
    case 'D':
      _VTG->mode = VTG::modeDGPS;
      break;
    case 'E':
      _VTG->mode = VTG::modeDR;
      break;
    case 'N':
      _VTG->mode = VTG::modeNotValid;
      break;
    case 'R':
      _VTG->mode = VTG::modeCoarsePosition;
      break;
    case 'S':
      _VTG->mode = VTG::modeSimulator;
      break;
    default:
      ERR();
    }

    break;

  case stateChecksum:
    chksum ^= _c; // undo checksum XOR

    if (_c == 0xd)
    {
      data->calculatedChecksum = chksum;
      data->messageChecksum = static_cast<u_int8_t>(strtol(match.c_str(), NULL, 16));
      state = stateCR;
      CLR();
      break;
    }

    if (!NUM(_c) || c > 2)
      ERR();

    match.push_back(_c);
    break;

  case stateCR:
    if (_c == 0xa)
    {
      *_sentence = data;
      data = NULL;
      init();

      if ((*_sentence)->calculatedChecksum != (*_sentence)->messageChecksum)
        return nmeaCompleteInvalid;

      return nmeaComplete;
    }

    ERR();
  }

  return nmeaIncomplete;
}
