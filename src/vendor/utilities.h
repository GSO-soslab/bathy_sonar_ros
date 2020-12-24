#if !defined __UTILITIES_H__
#define __UTILITIES_H__

#include <math.h>
#include <QString>

// *********
// Constants


// Degrees to radians conversions
static const double twopi = {6.28318530717958647693};
static const double dtor = twopi / 360.;		// degrees to radians conversion 
static const double rtod = 360. / twopi;		// radians to degrees conversion
static const double circledeg = 360.;			// degrees in a circle
static const double earthradius = 6378137.;		// earth's radius, metres
static const double earthcircum = twopi * earthradius;	// earth's circumference
static const double metrestodeg = circledeg / earthcircum; // metres to degrees longitude

static const double DEFAULT_SV = 1490.;		// default speed of sound

// End of constants
// ****************

// *******************
// Some handy short global functions

namespace utilities {

inline double square (const double a) 
{
	return a*a;
}

// calculate the hypotenuse of a right-angle triangle using Pythagoras
inline double pythag (const double x, const double y)
{
	return sqrt(x*x + y*y);
}

// calculate a side that is not a hypotenuse of a right-angle triangle using Pythagoras
inline double pythag_s (const double r, const double x)
{
	if (r < x)
		return 0.;
	else
		return sqrt(r*r - x*x);
}

// Set a flag in an unsigned short bitfield
inline void setFlagV(unsigned short& value, unsigned int flag, bool enabled)
{
    if (enabled)
    {
        value |= (1 << int(flag));
    }
    else
    {
        value &= ~(1 << int(flag));
    }
}

// Norma8lise an angle to +- 180 degrees
inline double norm180(const double angle)
{
	double ret_angle;
	ret_angle = fmod(angle, 360.);							// Force into +- 360
	ret_angle = (ret_angle < 0.) ? (ret_angle + 360.) : ret_angle;		// Force into 0 -> 360
	ret_angle = (ret_angle > 180.) ? (ret_angle - 360.) : ret_angle;	// Force into -180 -> +180
	return ret_angle;
}

// Normalise an angle to 0 -> 360 degrees
inline double norm360(const double angle)
{
	double ret_angle;
	ret_angle = fmod(angle, 360.);							// Force into +- 360
	ret_angle = (ret_angle < 0.) ? (ret_angle + 360.) : ret_angle;		// Force into 0 -> 360
	return ret_angle;
}

inline void AngleToSC(double angle, double& sang, double& cang)
{
	double rAngle = angle * dtor;		// Angle in radians, mathematical standard (anticlockwise)
	cang = cos(rAngle);
	sang = sin(rAngle);
}

// Set a flag in an unsigned short bitfield
inline void SetFlag(unsigned short& value, unsigned int flag, bool enabled)
{
	if (enabled) 
	{
        value |= (1 << flag);
	}
	else
	{
        value &= ~(1 << flag);
	}
}

// Get the state of a flag in an unsigned short bitfield
inline bool GetFlag(unsigned short value, unsigned int flag) 
{
    return !!(value & (1 << flag));
}

inline double DMStoDegs(int deg, int min, double sec) 
{
	return (abs(deg) + min / 60. + sec / 3600.) * (deg < 0. ? - 1 : 1);
}

inline double DMtoDegs(int deg, double min) 
{
	return (abs(deg) + min / 60.) * (deg < 0. ? - 1 : 1);
}

// Read and write from and to a char buffer
inline void SetUChar(unsigned char value, char *&pBuffer)		{*pBuffer++ = char(value);}
//inline void SetUShort(unsigned short value, char *&pBuffer)	{*((unsigned short*) pBuffer) = value; pBuffer+= sizeof(unsigned short);}
inline void SetUShort(unsigned short value, char *&pBuffer)     {*reinterpret_cast<unsigned short*>(pBuffer) = value; pBuffer+= sizeof(unsigned short);}
inline void SetShort(short value, char *&pBuffer)               {*reinterpret_cast<short*>(pBuffer) = value; pBuffer+= sizeof(short);}
inline void SetUInt(unsigned int value, char *&pBuffer)         {*reinterpret_cast<unsigned int*>(pBuffer) = value; pBuffer+= sizeof(unsigned int);}
inline void SetInt(int value, char *&pBuffer)					{*reinterpret_cast<int*>(pBuffer) = value; pBuffer+= sizeof(int);}
inline void SetLongLong(long long value, char *&pBuffer)		{*reinterpret_cast<long long*>(pBuffer) = value; pBuffer+= sizeof(long long);}
inline void SetFloat(float value, char *&pBuffer)				{*reinterpret_cast<float*>(pBuffer) = value; pBuffer+= sizeof(float);}
inline void SetDouble(double value, char *&pBuffer)             {*reinterpret_cast<double*>(pBuffer) = value; pBuffer+= sizeof(double);}
inline void SetBool(bool value, char *&pBuffer)                 {*pBuffer++ = (value ? 1 : 0);}
inline void SetBOOL(bool value, char *&pBuffer)                 {*pBuffer++ = (value ? 1 : 0);}

inline unsigned char	GetUChar(const char *&pBuffer)          {unsigned char value = *reinterpret_cast<const unsigned char*>(pBuffer); pBuffer+= sizeof(unsigned char); return value;}
inline unsigned short	GetUShort(const char *&pBuffer)         {unsigned short value = *reinterpret_cast<const unsigned short*>(pBuffer); pBuffer+= sizeof(unsigned short); return value;}
inline short			GetShort(const char *&pBuffer)          {short value = *reinterpret_cast<const short*>(pBuffer); pBuffer+= sizeof(short); return value;}
inline unsigned int		GetUInt(const char *&pBuffer)           {unsigned int value = *reinterpret_cast<const unsigned int*>(pBuffer); pBuffer+= sizeof(unsigned int); return value;}
inline int				GetInt(const char *&pBuffer)            {int value = *reinterpret_cast<const int*>(pBuffer); pBuffer+= sizeof(int); return value;}
inline long long		GetLongLong(const char *&pBuffer)       {long long value = *reinterpret_cast<const long long*>(pBuffer); pBuffer+= sizeof(long long); return value;}
inline float			GetFloat(const char *&pBuffer)          {float value = *reinterpret_cast<const float*>(pBuffer); pBuffer+= sizeof(float); return value;}
inline double			GetDouble(const char *&pBuffer)         {double value = *reinterpret_cast<const double*>(pBuffer); pBuffer+= sizeof(double); return value;}
inline bool				GetBool(const char *&pBuffer)           {unsigned char value = GetUChar(pBuffer); return (value == 0 ? false : true);}
}

// end of global functions
// *************

// ******************
// Assorted utilities

const int TimeStringLength = 30;

class VarUtilities
{
public:
    VarUtilities() {}

public:
	// Show a time interval as a string. 
	// Note: CSTime doesn't help here, as it assumes a time since 1970		
	// To get the years right, the routine would have to know the actual date.
	// Therefore, it uses a 365 day year.	
	static void TimeIntervalToString(char *string, int strlen, const double timeint);
	
    static void SetExt(QString& filename, const char* ext);	// Change the extension of a file path
};

// end of assorted utilities
// *************************

// ***********************
// General utility objects

// **************
// Position value
// Can be easting-northing or latitude-longitude
class CPosn
{
public:
	double E;		// easting or longitude
	double N;		// northing or latitude

	CPosn();																					// constructor
	CPosn(const double E1, const double N1);													// parameter constructor
    CPosn(QString lonString, QString latString);												// Constructs latitude and longitude from a pair of strings
	CPosn(int latDeg, int latMin, double latSec, int lonDeg, int lonMin, double lonSec);		// Constructs latitude and longitude from DMS
	CPosn(int latDeg, double latMin, int lonDeg, double lonMin);								// Constructs latitude and longitude from DM.M

    void Init(){E = 0.; N = 0.;}																// Initialise the object
    void Zero(){E = 0.; N = 0.;}																// Set the object to zero
	bool IsZero() const {return ((E == 0.) && (N == 0.));}										// Is the vector set to zero?
	CPosn &operator=(const CPosn &posn);														// Overloaded assignment operator
	void SetPos(const double E1, const double N1);												// Set the values
	void Translate(const double dist, const double angle);										// Translate by angle and distance
	bool IsInBox(const CPosn& llPos, const CPosn& urPos) const;									// Is the position inside a box defined by ll and ur corners?

	// linear distance between two positions
	double DistTo(const CPosn &posn2) {return (sqrt((E - posn2.E) * (E - posn2.E) + (N - posn2.N) * (N - posn2.N)));}

	// nautical bearing between two positions, from this point to point 2
	double PosAngle(const CPosn &posn2) {return (rtod*(atan2(posn2.E - E, posn2.N - N)));}

	// Sanity check, to trap corrupted data, infinities, NANs, etc.
	#define PosLim 1.E15
	bool IsPossible() const {return ((E > -PosLim) && (E < PosLim) &&	(N > -PosLim) && (N < PosLim));}

	// This structure can also be used to contain latitude and longitude : lat in 'N' and long in 'E'
	// If so used, format strings to show them
//    double GetDegsFromString(QString string);		// Get degrees from a string
    void SetLLFromString(QString lonString, QString latString);	// Set latitude and longitude from a pair of strings
	
	// Format to print angles in
	enum eAngleFormat{
				e_angle_d,			// Degrees and decimal degrees
				e_angle_dm,			// Degrees, minutes and decimal minutes
				e_angle_dms,		// Degrees, minutes, seconds and decimal seconds
				e_angle_raw			// The raw numbers with no degrees symbols, E/N, etc.
				};		
void PrintLL(QString &string, eAngleFormat format);		// Set up a string according to requested angle format
void PrintLat(QString &string, eAngleFormat format);	// Set up a string according to requested angle format
void PrintLon(QString &string, eAngleFormat format);	// Set up a string according to requested angle format

private:
    void PrintAngle(QString &string, double value, eAngleFormat format);	// Print one item, set the format
    void PrintDegs(QString &string, double value);			// Print one item, degrees & decimal degrees
    void PrintDegMins(QString &string, double value);		// Print one item, degrees, minutes & decimal minutes
    void PrintDegMinsSecs(QString &string, double value);	// Print one item, degrees, minutes, seconds & decimal seconds
    void PrintRaw(QString &string);							// Print lat & lon, just degrees and decimal degrees, no degrees symbols, etc.
};

double PosDist(const CPosn &posn1, const CPosn &posn2);		// linear distance between two positions
double PosAngle(const CPosn &posn1, const CPosn &posn2);	// nautical angle between two positions
double Dot(const CPosn &posn1, const CPosn &posn2);			// dot product of two position vectors

inline CPosn &CPosn::operator=(const CPosn &posn)	// Overloaded assignment operator
{
	E = posn.E;
	N = posn.N;
	return (*this);
}

inline void CPosn::SetPos(const double E1, const double N1)
{
	E = E1;
	N = N1;
}

// Translate a position by a distance and nautical angle
inline void CPosn::Translate(const double dist, const double angle)
{
	double angR = dtor * (90 - angle);
	E += dist * cos(angR);
	N += dist * sin(angR);
}

// Is the position inside a box defined by ll and ur corners?
inline bool CPosn::IsInBox(const CPosn& llPos, const CPosn& urPos) const
{
	return ((E > llPos.E) && (E < urPos.E) && (N > llPos.N) && (N < urPos.N));
}


inline CPosn operator +(const CPosn &posn1, const CPosn &posn2)	// Overloaded assignment operator
{
	return CPosn(posn1.E + posn2.E, posn1.N + posn2.N);
}

inline CPosn operator -(const CPosn &posn1, const CPosn &posn2)	// Overloaded assignment operator
{
	return CPosn(posn1.E - posn2.E, posn1.N - posn2.N);
}

inline CPosn operator *(const CPosn &posn1, const double &mul)	// Overloaded assignment operator
{
	return CPosn(posn1.E * mul, posn1.N * mul);
}

inline bool operator ==(const CPosn &posn1, const CPosn &posn2)
{
	return ((posn1.E == posn2.E) && (posn1.N == posn2.N));
}

inline bool operator !=(const CPosn &posn1, const CPosn &posn2)
{
	return (!(posn1 == posn2));
}

// linear distance between two positions
inline double PosDist(const CPosn &posn1, const CPosn &posn2)
{
	return (sqrt((posn1.E - posn2.E) * (posn1.E - posn2.E) + (posn1.N - posn2.N) * (posn1.N - posn2.N)));
}

// nautical bearing between two positions, from point 1 to point 2
inline double PosAngle(const CPosn &posn1, const CPosn &posn2)
{
	return (rtod*(atan2(posn2.E - posn1.E, posn2.N - posn1.N)));
}

// dot product of two position vectors
inline double Dot(const CPosn &posn1, const CPosn &posn2)
{
	return ((posn1.E * posn2.E) + (posn1.N * posn2.N));
}

// end of position object
// **********************

// Extended ships positions
class CShipPosn
{
public:
   CPosn m_posn;
   double m_cmg;					// course-made-good
	double m_speed;					// course-made-good speed
	
public:
	CShipPosn(void)		// Null constructor
	{
		m_cmg = 0.;
		m_speed = 0.;
	}
	
	~CShipPosn() {}		// Destructor
	
	CShipPosn(CPosn posn, double cmg, double speed)	// parameter constructor
	{
      m_posn  = posn;
      m_cmg   = cmg;
      m_speed = speed;
	}
	
	CShipPosn(const CShipPosn &posn)	// copy constructor
	{
      m_posn  = posn.m_posn;
      m_cmg   = posn.m_cmg;
      m_speed = posn.m_speed;
	}

	CShipPosn& operator = (const CShipPosn &posn)	// Overloaded assignment operator
	{
      m_posn  = posn.m_posn;
      m_cmg   = posn.m_cmg;
      m_speed = posn.m_speed;
		return (*this);
	}

	void Init()
	{
		m_posn.Init();
		m_cmg = 0.;
		m_speed = 0.;
	}	
};

inline CShipPosn operator + (const CShipPosn &posn1, const CShipPosn &posn2)
{
   return CShipPosn(posn1.m_posn - posn2.m_posn, posn1.m_cmg - posn2.m_cmg, posn1.m_speed - posn2.m_speed);
}

inline CShipPosn operator - (const CShipPosn &posn1, const CShipPosn &posn2)
{
   return CShipPosn(posn1.m_posn - posn2.m_posn, posn1.m_cmg - posn2.m_cmg, posn1.m_speed - posn2.m_speed);
}

// end of ship position object
// *************************
// ******************
// 3d position object

class C3dPos
{
public:
	double x;
	double y;
	double z;
	
public:
	C3dPos(void)		// Null constructor
	{
		x = 0.;
		y = 0.;
		z = 0.;
	}
	
	~C3dPos() {}		// Destructor
	
	C3dPos(double x1, double y1, double z1)	// parameter constructor
	{
		x = x1;
		y = y1;
		z = z1;
	}
	
	C3dPos(const C3dPos &posn)	// copy constructor
	{
		x = posn.x;
		y = posn.y;
		z = posn.z;
	}

	double Length() // Returns magnitude of C3dPos
	{
		return sqrt(x*x + y*y + z*z);
	}
	
    void setZero() {x = 0.; y = 0.; z = 0.;}
	bool IsZero() {return ((x == 0.) && (y == 0.) && (z == 0.));}

	C3dPos& operator = (const C3dPos &posn)	
	{
		x = posn.x;
		y = posn.y;
		z = posn.z;
		return (*this);
	}
	
	C3dPos& operator += (const C3dPos &posn)	
	{
		x += posn.x;
		y += posn.y;
		z += posn.z;
		return (*this);
	}
	
	C3dPos& operator -= (const C3dPos &posn)	
	{
		x -= posn.x;
		y -= posn.y;
		z -= posn.z;
		return (*this);
	}

	C3dPos& operator *= (const double mul)	
	{
		x *= mul;
		y *= mul;
		z *= mul;
		return (*this);
	}

	C3dPos operator * (const double mul) const	// Overloaded assignment operator
	{
		return C3dPos(x * mul, y * mul, x * mul);
	}


    bool operator == (const C3dPos& other) const {return (x == other.x) &&  (y == other.y) && (z == other.z);}

	// Get horizontal range
	// Warning: this is simply the horizontal distance to the origin, so not necessarily what is wanted for displays, etc.
	double HorizontalRange() const {return sqrt(x * x + y * y);}

	// Slant range, using all three dimensions
	double Range() const  {return sqrt(x * x + y * y + z * z);}

	// Slant range, using all x and z dimensions
	double RangeXZ() const  {return sqrt(x * x + z * z);}

	// Swap the x & y coordinates
	void SwapXY() {double tmp = x; x = y; y = tmp;}

	// Check to see if the position is inside a given rectangle; passing in lower-left and upper-right corners
	bool IsInRect(const C3dPos& ll, const C3dPos& ur) {return ((x >= ll.x) && (x <= ur.x) && (y >= ll.y) && (y <= ur.y));}

	/// Polar to Cartesian
	// This function is non-standard in the symmetry of the axis set, but easier to visualise in the context of
	// a bathymetric transducer.
	// We start with a range in the X axis, and rotate it in the direction of the Z axis (depth), using an angle that is
	// positive upwards. 
	void PolarToCartesian(double range, double angle)
	{
		double rAngle = -angle * dtor;
		x = range * cos(rAngle);
		y = 0.;
		z = range * sin(rAngle);
    }

	/// Rotation around Z axis in the XY plane. I.e., heading rotation.
	// Angle in degrees
	// Rotating clockwise, as per headings, rather than the mathematical standard of anticlockwise
	void RotateXY(double angle)
		{
			double rAngle = angle * dtor;		// Angle in radians, mathematical standard (anticlockwise)
			double cang = cos(rAngle);
			double sang = sin(rAngle);
			C3dPos posTmp;		// Temporary vector for storing results (otherwise the changed x is used in the calculation of the new y)
			posTmp.x = cang * x - sang * y;
			posTmp.y = sang * x + cang * y;
			posTmp.z = z;

			*this = posTmp;		// Update with the new vector
        }

	// As above, but with sine & cosine pre-computed (see AngleToSC())
	// Useful for processing many items with the same angle
	void RotateXY_SC(double sang, double cang)
		{
			C3dPos posTmp;		// Temporary vector for storing results (otherwise the changed x is used in the calculation of the new y)
			posTmp.x = cang * x - sang * y;
			posTmp.y = sang * x + cang * y;
			posTmp.z = z;

			*this = posTmp;		// Update with the new vector
        }

	/// Rotation around Y axis in the XZ plane. E.g. pitch.
	// Angle in degrees
	// Rotating clockwise, as per headings, rather than the mathematical standard of anticlockwise
	void RotateXZ(double angle)
		{
			double rAngle = angle * dtor;		// Angle in radians, mathematical standard (anticlockwise)
			double cang = cos(rAngle);
			double sang = sin(rAngle);
			C3dPos posTmp;		// Temporary vector for storing results 
			posTmp.x = cang * x - sang * z;
			posTmp.y = y;
			posTmp.z = sang * x + cang * z;

			*this = posTmp;		// Update with the new vector
        }

	// As above, but with sine & cosine pre-computed (see AngleToSC())
	// Useful for processing many items with the same angle
	void RotateXZ_SC(double sang, double cang)
		{
			C3dPos posTmp;		// Temporary vector for storing results 
			posTmp.x = cang * x - sang * z;
			posTmp.y = y;
			posTmp.z = sang * x + cang * z;

			*this = posTmp;		// Update with the new vector
		}	
	
	/// Rotation around X axis in the YZ plane. E.g. roll.
	// Angle in degrees
	// Rotating clockwise, as per headings, rather than the mathematical standard of anticlockwise
	void RotateYZ(double angle)
		{
			double rAngle = angle * dtor;		// Angle in radians, mathematical standard (anticlockwise)
			double cang = cos(rAngle);
			double sang = sin(rAngle);
			C3dPos posTmp;		// Temporary vector for storing results 
			posTmp.x = x;
			posTmp.y = cang * y - sang * z;
			posTmp.z = sang * y + cang * z;

			*this = posTmp;		// Update with the new vector
        }

	// As above, but with sine & cosine pre-computed (see AngleToSC())
	// Useful for processing many items with the same angle
	void RotateYZ_SC(double sang, double cang)
		{
			C3dPos posTmp;		// Temporary vector for storing results 
			posTmp.x = x;
			posTmp.y = cang * y - sang * z;
			posTmp.z = sang * y + cang * z;

			*this = posTmp;		// Update with the new vector
        }

	void AngleToSC(double angle, double& sang, double& cang)
	{
		double rAngle = angle * dtor;		// Angle in radians, mathematical standard (anticlockwise)
		cang = cos(rAngle);
		sang = sin(rAngle);
	}
};

inline C3dPos operator + (const C3dPos &posn1, const C3dPos &posn2)
{
	return C3dPos(posn1.x + posn2.x, posn1.y + posn2.y, posn1.z + posn2.z);
}

inline C3dPos operator - (const C3dPos &posn1, const C3dPos &posn2)
{
	return C3dPos(posn1.x - posn2.x, posn1.y - posn2.y, posn1.z - posn2.z);
}

// end of 3d position object
// *************************

// *********************************
// General offset from survey centre
class  CPosnOffset
{
public: 
	CPosnOffset();
//	void TextLog(QString& textlog);									// Log data to text

	enum ePointingDirection {ePort, eStbd, eFwd, eAft, eNone};		// Pointing direction, using azimuth value
	ePointingDirection GetPointingDirection() const;	
		
	// for MRU; swaps roll,pitch & heading if azimuth is not 0. 
	// Currently only works for steps of 90 degrees.
	// Returns true if a change has been made, else false.	
	bool RotateAzimuth(double& roll, double& pitch, double& heading) const;	

public: 
    double m_height;
    double m_forward;
    double m_starboard;
    double m_azimuth;
    double m_elevation;
    double m_skew;
    double m_time;
    double m_water_depth;
    double m_pitch;
    bool   m_inverted;			// The sensor is mounted upside-down: invert its angles
};

// end of CPosnOffset
// ******************

// **********
// Time value

// A bit like CTime, except that it only works in UTC


const long int CSTimeSToUS = 1000000;					// seconds to microseconds
const double CSTimeUSToS = 1. / double(CSTimeSToUS);

/*
class CSTime
{
public:
	double m_time;		// seconds since 1970
	
public:
	// Constructors
	
	CSTime();																			// Null constructor: sets the time to zero.
	CSTime(double time); 																// Sets a time in seconds
	CSTime(long int sec, long int usec);												// Converts from separate seconds and microseconds.
	CSTime(int hour, int min, int sec, long int usec, int day, int month, int year);	// Converts from separate values.
	CSTime(int hour, int min, double sec, int day, int month, int year);				// Converts from separate values.
	CSTime(char* string);																// Converts from a string
	~CSTime();
	
	// Get the time value in various ways
	double GetTime() const {return m_time;}																						// As a double
	unsigned long GetSeconds() {return (unsigned long) m_time;}																	// Seconds part. Lasts until the year 2109.
	unsigned long GetNearestSecond() {return (unsigned long) (m_time + 0.5);}													// Round to the nearest second
	unsigned long GetmicroSeconds() {double sec; double fsec = modf(m_time, &sec); return (unsigned long) (fsec * 1e6);}		// Microseconds part
	unsigned long GetmilliSeconds() {double sec; double fsec = modf(m_time, &sec); return (unsigned long) (fsec * 1e3);}		// Milliseconds part

	bool IsZero() {return m_time == 0.;}		// Returns true if the time is (still) zero

	// ************
	// Set the time
	// ************

	// Set the current time from a set of ints
	void SetTime(double time) {m_time = time;}
	void IntToTime(int hour, int min, int sec, long int usec, int day, int month, int year);
	void IntToTime1(int hour, int min, int sec, int day, int month, int year);
	void IntToTime2(int hour, int min, int sec, int yday, int year);
	void IntToTime3(int hour, int min, double sec, int day, int month, int year);
	void IntToTime4(int hour, int min, double sec, int yday, int year);
	
	// Set the current time from seconds and microseconds
	void LIntToTime(unsigned long sec, long int usec) {m_time = (double) sec + (double) usec * CSTimeUSToS;}

	// Set the current time from a string
	void StrToTime(char* string);
	
	// Set the current time from a string (no seconds)
	void StrToTime2(char* string);
	
	// Set the current time from a string (fractional seconds)
	void StrToTime3(char* string);

	// Set the time from the custom time and date string. 
	bool TimeDateStrToTime(const char* timeStr, const char* dateStr);

	// Set the time from a custom time string. This format takes time without date,
	// so we have to get that from the PC clock, as fed in from the original data strings
	// Passes the corrected time back by reference, and returns good or bad status
	bool TimeStrToTime(const char* timeStr, CSTime& origtime);

	// Set the time from the GPS time string. NMEA strings don't include date,
	// so we have to get that from the PC clock, as fed in from the	 original data strings
	// Passes the corrected time back by reference, and returns good or bad status
	bool TimeFromGPSTime(double nmeagpstime, CSTime& origtime);

	// Set the time from the GPS time string and day, month and year
	// Returns good or bad status
	bool TimeFromGPSTimeDMY(const double nmeagpstime, const int day, const int month, const int year);

	// Set the time from the GPS time and date string 
	// Returns good or bad status
	bool TimeFromGPSTimeDate(const double nmeagpstime, const long nmeagpsdate);

	// *******************************
	// Get the time in various formats
	// *******************************

	// Return a set of ints from the current time
	void TimeToInt(int& hour, int& min, int& sec, long int& usec, int& day, int& month, int& year, int& yday) const;
	
	// Return a set of ints from the current time: no microseconds
	void TimeToInt1(int& hour, int& min, int& sec, int& day, int& month, int& year, int& yday) const;

	// Return a set of ints from the current time with wday
	void TimeToInt2(int& hour, int& min, int& sec, int& day, int& month, int& year, int& yday, int& wday) const;

	// Return a set of ints from the current time with wday and usec
	void TimeToInt3(int& hour, int& min, int& sec, long int& usec, int& day, int& month, int& year, int& yday, int& wday) const;
	
	// Return a pair of long ints from the current time; seconds and microseconds
	void TimeToLInt(long int& sec, long int& usec) const;
	void TimeToLIntU(unsigned long int& sec, int& usec) const {TimeToLInt((long int&) sec, (long int&) usec);}
	
	// Return unsigned long int and int from the current time; seconds and microseconds
	void TimeToLInt2(unsigned long int& sec, int& usec) const;
	
	// Return a pair of long ints from the current time ; seconds and milliseconds
	void TimeToLIntMs(long int& sec, long int& msec) const;
	
	// Return a string, set from the current time.
	// "%02d:%02d:%02d %02d/%02d/%04d", hour, min, sec, day, month, year
	enum {TimeToStrLen = 20};
	void TimeToStr(char *string, int strlen) const;

	// Return a string, set from the current time: show decimal seconds, but not date
	// %02d:%02d:%08.5f", hour, min, dsec
	enum {TimeToStr1Len = 15};
	void TimeToStr1(char *string, int strlen) const;

	// Return a string, set from the current time.(no seconds)
	// "%02d:%02d %02d/%02d/%04d", hour, min, day, month, year
	enum {TimeToStr2Len = 17};
	void TimeToStr2(char *string, int strlen) const;

	// Return a string, set from the current time: show decimal seconds and date
	// "%02d:%02d:%08.5f %02d/%02d/%04d", hour, min, dsec, day, month, year
	enum {TimeToStr3Len = 27};
	void TimeToStr3(char *string, int strlen) const;

	// Return a string, set from the current time.  Date first
	// "%02d/%02d/%04d %02d:%02d:%02d", day, month, year, hour, min, sec
	enum {TimeToStr4Len = 20};
	void TimeToStr4(char *string, int strlen) const;

	// Return an NMEA-type time string, set from the current time (HHMMSS.SSS)
	enum {NEMAStringLength = 12};
	void TimeToNMEAString(char *string) const;

	// Return the time in seconds as a string
	enum {SecondsStringLength = 15};		// 9 digits, 5 decimal places = 15
	void TimeToSecondsString(char *string, int strlen) const;

	// ***********************************
	// Manipulate the time in various ways
	// ***********************************
		
	void RoundToDay();												// Round down to the start of the current day
	void TimeOfDay();												// Compute just the time in the current day
	void SetTimeOfDay(int hour, int min, int sec, long int usec);	// Set the time from just the time of day: use date from existing and time from the input parameters
	void SetTimeOfDay(int hour, int min, double sec);				// Set the time from just the time of day: use date from existing and time from the input parameters
	void SetTimeOfDay(CSTime& dayTime);								// Set the time from just the time of day: use date from existing and time from the input parameters
	
	// ************
	// Validatation
	// ************

	// Check a set of time integer codes
    bool ValidateTimeInt(int hour, int min, int sec, long int usec, int day, int month, int year) const;
    bool ValidateTimeInt(int hour, int min, int sec, long int usec, int yday, int year) const;

	// *****************************
	// Time to and from the PC clock
	// *****************************

	// Get the current PC time
	void TimeNow();

	// Set the PC clock 
//	bool SetPCClock();

	// ********************
	// GPS time corrections
	// ********************
	void SetFromGPSTimeOfWeek(unsigned long GPSTimeOfWeek);			// Set the time from a GPS time-of-week value, given that the current time is approximately correct

	// *********
	// Operators
	// *********

	CSTime &operator=(const CSTime &time)	// Overloaded assignment operator
	{
		m_time = time.m_time;
		return (*this);
	}
		
	CSTime &operator=(const double &time)	// Overloaded assignment operator
	{
		m_time = time;
		return (*this);
	}
	
	CSTime &operator+=(const CSTime &time)	// Overloaded assignment operator
	{
		m_time += time.m_time;
		return (*this);
	}
	
	CSTime &operator-=(const CSTime &time)	// Overloaded assignment operator
	{
		m_time -= time.m_time;
		return (*this);
	}
	
	
	CSTime &operator*=(const double &mul)	// Overloaded assignment operator
	{
		m_time *= mul;
		return (*this);
	}
	
	CSTime &operator/=(const double &div)	// Overloaded assignment operator
	{
		m_time /= div;
		return (*this);
	}
	
};

inline CSTime operator +(const CSTime &time1, const CSTime &time2)
{
	return CSTime(time1.m_time + time2.m_time);
}

inline CSTime operator -(const CSTime &time1, const CSTime &time2)
{
	return CSTime(time1.m_time - time2.m_time);
}

inline CSTime operator *(const CSTime &time1, const double &mul)
{
	return CSTime(time1.m_time * mul);
}

inline CSTime operator /(const CSTime &time1, const double &div)
{
	return CSTime(time1.m_time / div);
}

inline bool operator <(const CSTime &time1, const CSTime &time2)
{
	return (time1.m_time < time2.m_time);
}

inline bool operator >(const CSTime &time1, const CSTime &time2)
{
	return (time1.m_time > time2.m_time);
}

inline bool operator ==(const CSTime &time1, const CSTime &time2)
{
	return (time1.m_time == time2.m_time);
}


inline bool operator <(const CSTime &time1, const double &time)
{
	return (time1.m_time < time);
}

inline bool operator >(const CSTime &time1, const double &time)
{
	return (time1.m_time > time);
}

inline bool operator ==(const CSTime &time1, const double &time)
{
	return (time1.m_time == time);
}

// Return the difference of two CSTime objects as a double
inline double timeDiff(const CSTime &time1, const CSTime &time2)
{
	return (time1.m_time - time2.m_time);
}

// Set the current time from seconds and microseconds, without using a CSTime object
inline double LIntToDTime(long int sec, long int usec) 
{
	return ((double) sec + (double) usec * CSTimeUSToS);
}


// Default time format string
#define CTSTimeStrLen   20
#define CTSTimeStrLen1  15				// for TimeToStr1
#define CTSTimeStrLen2  17				// for TimeToStr2
#define CTSTimeStrLen3  26				// for TimeToStr3
#define CTSTimeFmtStr  %02d:%02d:%02d %02d/%02d/%04d // hh:mm:ss dd/mm/yyyy

// End of time value
// *****************

// **************************************************************************
// Time reference tool. Uses a time stamp and clock ticks to store clock time
// **************************************************************************
class CSTimeRef
{
public:
	CSTimeRef() {Reset();SetToPCTime();}		// Initialise to the PC clock time
	~CSTimeRef() {;}
	
	void Reset() {m_timeStamp = 0.; m_tickCountAtStamp = 0; m_writeLock = false;}		// Return to zero values
	bool IsSet() {return !(m_timeStamp.IsZero());}					// Returns true if a time stamp has been entered

	bool SetTime(double time);			// Set a time in seconds. Returns false if fails, probably because write lock is set
	bool SetTime(CSTime& time);			// Set a time as a CSTime. Returns false if fails, probably because write lock is set
	bool SetToPCTime();					// Set the reference to the time of the PC clock

	const CSTime GetTime();				// Get the time as a CSTime, which can then be accessed in various ways
    double GetTimeD();                  // Get the time as a double, Unix time seconds

private:
	CSTime m_timeStamp;					// The time set into the class
    unsigned long int m_tickCountAtStamp;			// The tick count when the stamp is set
	bool m_writeLock;					// Prevent writing while another process is reading or writing for thread safety
};
*/

/*
// ************
// PPS handling
// ************

class CPPSProc
{
public:
	CPPSProc() : m_pPPS(nullptr) {;}
	CPPSProc(unsigned char* pPPS) {m_pPPS = pPPS;}

	unsigned char* m_pPPS;

	enum PPSdecode
	{
								// 0 / 1
		PPS_DISABLE_BIT			= 0x1,		// 1PPS is enabled / 1PPS is disabled
		PPS_EDGE_BIT			= 0x2,		// 1PPS acts on rising edge	/ 1PPS acts on falling edge
		PPS_ACKNOWLEDGE_BIT		= 0x4,		// TEM has not received 1PPS / TEM has received 1PPS
		PPS_PERIOD_ERROR_BIT	= 0x8,		// PPS period matches TEM clock / PPS period does not match TEM clock
		PPS_USE_PC_TIME			= 0x10		// Ignore TEM time altogether and use the PC clock instead
	};

	bool PPSEnabled()			const	{return (((*m_pPPS) & PPS_DISABLE_BIT) == 0);}
	void EnablePPS(bool enable)			{if (enable) (*m_pPPS) &= ~PPS_DISABLE_BIT; else (*m_pPPS) |= PPS_DISABLE_BIT;}
	bool PPSRisingEdge()		const	{return (((*m_pPPS) & PPS_EDGE_BIT) == 0);}
	void SetPPSEdge(bool rising)		{if (rising) (*m_pPPS)	&= ~PPS_EDGE_BIT; else (*m_pPPS) |= PPS_EDGE_BIT;}
	bool UsePCTime()			const	{return (((*m_pPPS) & PPS_USE_PC_TIME) != 0);}							// Warning; the sense of this has got inverted at some point; it returns true if the sonar is set to sonar clock
	void SetUsePCTime(bool usePCTime)	{ if (usePCTime) (*m_pPPS) |= PPS_USE_PC_TIME; else (*m_pPPS)	&= ~PPS_USE_PC_TIME;}
	bool PPSAcknowledged()		const	{return (((*m_pPPS) & PPS_ACKNOWLEDGE_BIT) != 0);}
	void SetPPSAcknowledged(bool received) {if (received) (*m_pPPS) |= PPS_ACKNOWLEDGE_BIT; else (*m_pPPS) &= ~PPS_ACKNOWLEDGE_BIT;}
	bool PPSPeriodError()		const	{return (((*m_pPPS) & PPS_PERIOD_ERROR_BIT) != 0);}
	void SetPPSPeriodError(bool error)	{if (error) (*m_pPPS) |= PPS_PERIOD_ERROR_BIT; else (*m_pPPS) &= ~PPS_PERIOD_ERROR_BIT;}
};
*/

// **********
// File names
// **********

/*
class CSFileName
{
public:
    CSFileName(QString name) {m_name = name;}
    void SetName(QString name) {m_name = name;}
	void SetName(char* name) {m_name = name;}
    QString GetName() {return m_name;}
    QString GetExt();								// Get the extension
    QString SetExt(const QString ext);				// Change the extension (returns the new name)
    QString SetExt(const char* ext) {QString extStr = ext; SetExt(extStr);}

private:
    QString m_name;			// File name string
};
*/

#endif		// __UTILITIES_H__
