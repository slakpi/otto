#ifndef NMEA_HPP
#define NMEA_HPP

#include <ctime>

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
		nmeaGLL,
		nmeaGSA,
		nmeaGSV,
		nmeaRMC,
		nmeaVTG
	};

	class NMEABase
	{
	public:
		NMEABase(NMEAType _type);

	public:
		~NMEABase();

	public:
		NMEAType getType() const;

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
		~GGA();

	public:
		time_t utc;
		double latDeg;
		double latMin;
		bool south;
		double lonDeg;
		double lonMin;
		bool west;
		FixIndicator fix;
		int satellites;
		double HDOP;
		double altMSL; //always meters
		double geoidSep; //always meters
		double diffCorrAge;
		int diffRefID;
	};

private:
	enum ParseState
	{
		stateInitial,
		stateId,

		stateGGAUTC,
		stateGGALatDeg,
		stateGGALatMin,
		stateGGALatIndicator,
		stateGGALonDeg,
		stateGGALonMin,
		stateGGALonIndicator,
		stateGGAFixIndicator,
		stateGGA
	};

public:
	NMEA();

public:
	~NMEA();

public:
	void init();

	ParseStatus putChar(char _c, NMEABase **_sentence);
};

#endif
