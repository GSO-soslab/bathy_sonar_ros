#include <time.h>
#include "utilities.h"
//#include  <sys/types.h>
//#include  <sys/timeb.h>
#include <QString>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// General utility objects

using namespace utilities;

// ***************
// Position object
CPosn::CPosn()	// Null constructor
{
    E = 0.;
    N = 0.;
}

CPosn::CPosn(const double E1, const double N1)	// Parameter constructor
{
    E = E1;
    N = N1;
}

CPosn::CPosn(QString lonString, QString latString)	// Constructs latitude and longitude from a pair of strings
{
//    SetLLFromString(lonString, latString);
}

// Constructs latitude and longitude from DMS
CPosn::CPosn(int latDeg, int latMin, double latSec, int lonDeg, int lonMin, double lonSec)
{
    E =	DMStoDegs(lonDeg, lonMin, lonSec);			// longitude
    N =	DMStoDegs(latDeg, latMin, latSec);			// latitude
}

// Constructs latitude and longitude from DM.M
CPosn::CPosn(int latDeg, double latMin, int lonDeg, double lonMin)
{
    E =	DMtoDegs(lonDeg, lonMin);			// longitude
    N =	DMtoDegs(latDeg, latMin);			// latitude
}

// This structure can also be used to contain latitude and longitude : lat in 'N' and long in 'E'
// If so used, format strings to show them

// Set up a string according to requested angle format
void CPosn::PrintLL(QString &string, eAngleFormat format)
{
    if (format == e_angle_raw)
    {
        PrintRaw(string);
    }
    else
    {
        QString string2;
        PrintLat(string2, format);	// Latitude
        string += string2;
        string += " ";
        PrintLon(string2, format);	// Longitude
        string += string2;
    }
}

// Set up a string according to requested angle format
void CPosn::PrintLat(QString &string, eAngleFormat format)
{
    PrintAngle(string, N, format);
    string += " ";
    if (N > 0.)
    {
        string += "N";
    }
    else
    {
        string += "S";
    }
}

// Set up a string according to requested angle format
void CPosn::PrintLon(QString &string, eAngleFormat format)
{
    PrintAngle(string, E, format);
    string += " ";
    if (E > 0.)
    {
        string += "E";
    }
    else
    {
        string += "W";
    }
}

// Print one item, set the format
void CPosn::PrintAngle(QString &string, double value, eAngleFormat format)
{
    switch (format)
    {
        default:
        case e_angle_d:
            PrintDegs(string, value);
            break;
        case e_angle_dm:
            PrintDegMins(string, value);
            break;
        case e_angle_dms:
            PrintDegMinsSecs(string, value);
            break;
    }
}

// Print one item, degrees & decimal degrees
void CPosn::PrintDegs(QString &string, double value)
{
    value = fabs(value);			// keep positive: the calling routine works out E/W, N/S
    string.sprintf("%016.14f�", value);
}

// Print one item, degrees, minutes & decimal minutes
void CPosn::PrintDegMins(QString &string, double value)
{
    int degs;
    double mins;
    
    value = fabs(value);			// keep positive: the calling routine works out E/W, N/S
    degs = int(value);
    mins = (value - degs) * 60.;
    
    string.sprintf("%03d� %011.9f'", degs, mins);
}

// Print one item, degrees, minutes, seconds & decimal seconds
void CPosn::PrintDegMinsSecs(QString &string, double value)
{
    int degs;
    int mins;
    double mins_f;
    double secs;
    
    value = fabs(value);			// keep positive: the calling routine works out E/W, N/S
    degs = int(value);
    mins_f = (value - degs) * 60.;
    mins = int(mins_f);
    secs = (mins_f - mins) * 60.;
    
    string.sprintf("%03d� %02d' %07.5f\"", degs, mins, secs);
}

// Print lat & lon, just degrees and decimal degrees, no degrees symbols, etc.
void CPosn::PrintRaw(QString &string)
{
    string.sprintf("%016.14f %016.14f", N, E);
}

// TODO: convert to QString functions
/*
// Get degrees from a string, which could be:
// dd.dddddd
// dd mm.mmmm
// dd mm ss.ssss
// minutes and seconds markers (' ", '') may or may not be present
// the string may also contain 'N', 'E', 'S' or 'W'. If so, change the sign as appropriate.
// ditto if there is a leading minus sign
double CPosn::GetDegsFromString(QString string)
{
    QString token;
    double degrees = 0.;
    double minutes = 0.;
    double seconds = 0.;
    int tokenLoop = 0;
    int currPos = 0;
    //char* tokenChar = NULL;

    token = string.Tokenize("\' \"�-NESW", currPos);
    while (token != "")
    {
        switch (tokenLoop++)
        {
        case 0:
            sscanf_s((LPCTSTR) token, "%lf", &degrees);
            break;

        case 1:
            sscanf_s((LPCTSTR) token, "%lf", &minutes);
            break;

        case 2:
            sscanf_s((LPCTSTR) token, "%lf", &seconds);
            break;
        }

        // Get the next one
        token = string.Tokenize("\' \"", currPos);
    }
	
    degrees = degrees + minutes / 60. + seconds / 3600.;

    // could be negative
    if (string.FindOneOf("-SW") != -1)
    {
        degrees = -degrees;
    }

    return degrees;
}


// Set latitude and longitude from a pair of strings
void CPosn::SetLLFromString(QString lonString, QString latString)
{
    E = GetDegsFromString(lonString);
    N = GetDegsFromString(latString);
}
*/


// end of position object
// **********************

// *********************************
// General offset from survey centre

CPosnOffset :: CPosnOffset()
{
    m_height = 0.;
    m_forward = 0.;
    m_starboard = 0.;
    m_azimuth = 0.;
    m_elevation = 0.;
    m_skew = 0.;
    m_time = 0.;
    m_water_depth = 0.;
    m_pitch = 0.;
    m_inverted = false;
}

// Pointing direction, using azimuth value
CPosnOffset::ePointingDirection CPosnOffset::GetPointingDirection() const
{
    ePointingDirection direction;
    double azimuth360 = norm360(m_azimuth);				// Get the angle in the range 0 > 360.
    if ((azimuth360 < 45.) || (azimuth360 > 315.))
        direction = eFwd;
    else if ((azimuth360 >= 45.) && (azimuth360 <= 135.))
        direction = eStbd;
    else if ((azimuth360 > 125.) && (azimuth360 < 225.))
        direction = eAft;
    else
        direction = ePort;
    
    return direction;
}

// for MRU; swaps roll,pitch & heading if azimuth is not 0.
// Currently only works for steps of 90 degrees. TODO: allow other azimuth offsets.
// Returns true if a change has been made, else false.
bool CPosnOffset::RotateAzimuth(double& roll, double& pitch, double& heading) const
{
    double rollIn = roll;
    double pitchIn = pitch;
    bool changed = false;
    
    if (m_azimuth == 0.)
    {
        return false;				// Nothing to do
    }
    else if (m_azimuth == 90.)
    {
        roll = pitchIn;
        pitch = -rollIn;
        heading = norm360(heading + m_azimuth);
        changed = true;
    }
    else if ((m_azimuth == 180.) || (m_azimuth == -180.))
    {
        roll = -rollIn;
        pitch = -pitchIn;
        heading = norm360(heading + m_azimuth);
        changed = true;
    }
    else if ((m_azimuth == -90) || (m_azimuth == 270.))
    {
        roll = -pitchIn;
        pitch = rollIn;
        heading = norm360(heading + m_azimuth);
        changed = true;
    }
    
    // Other angles aren't processed (TODO: work out the 3D geometry of the whole thing ...)
    return changed;
}


// end of CPosnOffset
// ******************

// ***********
// Time object

// TODO: convert to Qt methods
/*
CSTime::CSTime()
{
    m_time = 0.;
}

CSTime::CSTime(double time)
{
    m_time = time;
}

CSTime::CSTime(long int sec, long int usec)
{
    LIntToTime(sec, usec);
}

CSTime::CSTime(int hour, int min, int sec, long int usec, int day, int month, int year)
{
    IntToTime(hour, min, sec, usec, day, month, year);
}

CSTime::CSTime(int hour, int min, double sec, int day, int month, int year)
{
    IntToTime3(hour, min, sec, day, month, year);
}

CSTime::CSTime(char* string)
{
    StrToTime(string);
}

CSTime::~CSTime()
{
}

// Convert the current time from a set of ints to the Local Time
// Sets time to zero if fed an invalid set of integer codes
// treats input time as UTC and converts to Local
// therefore if input time is local time it will be converted to UTC
void CSTime::IntToTime(int hour, int min, int sec, long int usec, int day, int month, int year)
{
    struct tm tm_s;				// tm structure
    tm_s.tm_isdst = 0;			// no daylight saving
    tm_s.tm_wday = 0;
    tm_s.tm_yday = 0;
	
    if (ValidateTimeInt(hour, min, sec, usec, day, month, year))
    {
        tm_s.tm_hour = hour;
        tm_s.tm_min = min;
        tm_s.tm_sec = sec;
        tm_s.tm_mday = day;
        tm_s.tm_mon = month - 1;	// tm encodes 0-11. Normal useage is 1-12.
        tm_s.tm_year = year - 1900;	// tm encodes as year - 1900
        tm_s.tm_isdst = 0;			// no daylight saving
        m_time = (double) _mkgmtime(&(tm_s)) + (double) usec * CSTimeUSToS;
    }
    else
    {
        m_time = 0.;
    }
}

void CSTime::IntToTime1(int hour, int min, int sec, int day, int month, int year)
{
    IntToTime(hour, min, sec, 0, day, month, year);
}

void CSTime::IntToTime2(int hour, int min, int sec, int yday, int year)
{
    struct tm tm_s;				// tm structure
    int usec = 0;
	
    if (ValidateTimeInt(hour, min, sec, 0, yday, year))
    {
        tm_s.tm_hour	= hour;
        tm_s.tm_min		= min;
        tm_s.tm_sec		= sec;
        tm_s.tm_mday	= yday;			// _mkgmtime() can't compute m_time from yday, but it can be fudged by putting the year day as the day of the month. Both month day and *our* Julian day start at 1. (tm_yday starts at 0)
        tm_s.tm_mon		= 0;
        tm_s.tm_wday	= 0;
        tm_s.tm_yday	= 0;
        tm_s.tm_year	= year - 1900;	// tm encodes as year - 1900
        tm_s.tm_isdst	= 0;			// no daylight saving
        m_time = (double) _mkgmtime(&(tm_s)) + (double) usec * CSTimeUSToS;
    }
    else
    {
        m_time = 0.;
    }
}

void CSTime::IntToTime3(int hour, int min, double sec, int day, int month, int year)
{
    long nSec = (long) sec;
    long usec = (long) ((sec - nSec) * 1e6);
    IntToTime(hour, min, nSec, usec, day, month, year);
}

void CSTime::IntToTime4(int hour, int min, double sec, int yday, int year)
{
    IntToTime2(hour, min, (int)sec, yday, year);
    if (m_time != 0.)
    {
        m_time += sec - (int)sec;
    }
}


// Check a set of time integer codes
bool CSTime::ValidateTimeInt(int hour, int min, int sec, long int usec, int day, int month, int year) const
{
    // Bare checks
    if (((hour < 0) || (hour > 23)) ||
        ((min < 0) || (min > 59)) ||
        ((sec < 0) || (sec > 59)) ||
        ((usec < 0) || (usec > CSTimeSToUS)) ||
        ((day < 1) || (day > 31)) ||
        ((month < 1) || (month > 12)) ||
        ((year < 1970) || (year > 2099)))
        return false;
	
    // Day-month-year checks 30 days hath september ...
    if (((month == 4) || (month == 6) || (month == 9) || (month == 11))
        && (day > 30))
        return false;
	
    // ... excepting when it's Chris's birthday ...
    if (month == 2)
    {
        if ((year % 4) == 0)
        {
            if (day > 29)
                return false;
        }
        else
        {
            if (day > 28)
                return false;
        }
    }
    return true;
}

bool CSTime::ValidateTimeInt(int hour, int min, int sec, long int usec, int yday, int year) const
{
    bool status = false;
    if (ValidateTimeInt(hour, min, sec, usec, 1, 1, year))
    {
        status = ((yday >= 0) && (yday <= 365));
    }
    return status;
}

// Round down to the start of the current day
void CSTime::RoundToDay()
{
    int hour, min, sec, day, month, year, yday;
    long usec;
    TimeToInt(hour, min, sec, usec, day, month, year, yday);	// Get the separate parts
    IntToTime(0, 0, 0, 0, day, month, year);					// Zero the time of day
}

// Compute just the time in the current day
void CSTime::TimeOfDay()
{
    int hour, min, sec, day, month, year, yday;
    long usec;
    TimeToInt(hour, min, sec, usec, day, month, year, yday);	// Get the separate parts
    IntToTime(hour, min, sec, usec, 0, 0, 0);					// Zero the day, month, year part
}

// Set the time from just the time of day: use date from existing and time from the input parameters
void CSTime::SetTimeOfDay(int hour, int min, int sec, long int usec)
{
    int hour1, min1, sec1, day, month, year, yday;
    long usec1;
    TimeToInt(hour1, min1, sec1, usec1, day, month, year, yday);	// Get the separate parts

    // If the current time is close to the end of the day, and the new time is close to the start of the day, increment by one day
    const int minutesMargin = 10;									// This close to midnight to indicate a roll-over
    double timeOffset = 0.;											// Offset to add to time to shift by one day
    if ((hour1 == 23) && (min1 >= (60 - minutesMargin)) && (hour == 0) && (min <= minutesMargin))
        timeOffset = 24 * 60 * 60;									// Length of a day in seconds
    if ((hour1 == 0) && (min1 <= minutesMargin) && (hour == 23) && (min >= (60 - minutesMargin)))	// Similar for under-runs
        timeOffset = -(24 * 60 * 60);								// Length of a day in seconds

    IntToTime(hour, min, sec, usec, day, month, year);				// Day, month, year from current, hour, minutes, seconds from input
    m_time += timeOffset;											// Add the day offset if any
}

// Set the time from just the time of day: use date from existing and time from the input parameters
void CSTime::SetTimeOfDay(int hour, int min, double sec)
{
    long nSec = (long) sec;
    long usec = (long) ((sec - nSec) * 1e6);
    SetTimeOfDay(hour, min, nSec, usec);
}

// Set the time from just the time of day: use date from existing and time from the input parameters
void CSTime::SetTimeOfDay(CSTime& dayTime)
{
    int hour, min, sec, day, month, year, yday;
    long usec;
    dayTime.TimeToInt(hour, min, sec, usec, day, month, year, yday);	// Get the separate parts
    SetTimeOfDay(hour, min, sec, usec);
}

// Set the current time from a string
void CSTime::StrToTime(char* string)
{
    if (string != NULL)
    {
        int hour, min, sec, day, month, year;
		
        if (sscanf_s(string, "%d:%d:%d %d/%d/%d", &hour, &min, &sec, &day, &month, &year) == 6)
            IntToTime1(hour, min, sec, day, month, year);
        else
            m_time = 0;			// safety
    }
}

// Set the current time from a string: no seconds
void CSTime::StrToTime2(char* string)
{
    if (string != NULL)
    {
        int hour, min, sec, day, month, year;
        sec = 0;
		
        if (sscanf_s(string, "%d:%d %d/%d/%d", &hour, &min, &day, &month, &year) == 5)
            IntToTime1(hour, min, sec, day, month, year);
        else
            m_time = 0;			// safety
    }
}

// Read in time as a string, in the format HH:MM:SS.SSSS DD/MM/YYYY
void CSTime::StrToTime3(char* string)
{
    if (string != NULL)
    {
        int hour, min, sec, day, month, year;
        long usec;
        float dsec;
		
        if (sscanf_s(string, "%d:%d:%f %d/%d/%d", &hour, &min, &dsec, &day, &month, &year) == 6)
        {
            sec = (int)dsec;
            usec = ((long int) (dsec * CSTimeSToUS)) % CSTimeSToUS; // Get the microseconds
            IntToTime(hour, min, sec, usec, day, month, year);
        }
        else
        {
            m_time = 0;
        }
    }
}

bool CSTime::TimeDateStrToTime(const char* timeStr, const char* dateStr)
{
    bool status = false;
    if ((timeStr != NULL) && (dateStr != NULL))
    {
        int hour, min, sec, day, month, year;
        long usec,msec,nmeadate;
        float dsec;
        double nmeatime;
		
        if ((sscanf_s(timeStr, "%d:%d:%f", &hour, &min, &dsec) == 3) &&
            (sscanf_s(dateStr, "%d/%d/%d", &day, &month, &year) == 3))
        {
            sec = (int)dsec;
            usec = ((long int) (dsec * CSTimeSToUS)) % CSTimeSToUS; // Get the microseconds
            IntToTime(hour, min, sec, usec, day, month, year);
            status = true;
        }
        else if ((sscanf_s(timeStr, "%d:%d:%d", &hour, &min, &sec) == 3) &&
            (sscanf_s(dateStr, "%d/%d/%d", &day, &month, &year) == 3))
        {
            sec = (int)dsec;
            usec = ((long int) (dsec * CSTimeSToUS)) % CSTimeSToUS; // Get the microseconds
            IntToTime1(hour, min, sec, day, month, year);
            status = true;
        }
        else if ((sscanf_s(timeStr, "%2d%2d%2d.%3d", &hour, &min, &sec, &msec) == 4) &&
            (sscanf_s(dateStr, "%d/%d/%d", &day, &month, &year) == 3))
        {
            usec = msec*1000; // Get the microseconds
            IntToTime(hour, min, sec, usec, day, month, year);
            status = true;
        }
        else if ((sscanf_s(timeStr, "%lf", &nmeatime) == 1) &&
            (sscanf_s(dateStr, "%ld", &nmeadate) == 1))
        {
            status = this->TimeFromGPSTimeDate(nmeatime, nmeadate);
            if (!status)
            {
                CSTime origtime = *this;
                status = this->TimeFromGPSTime(nmeatime, origtime);
            }
        }
    }
    return status;
}
// set time from a time string only.
bool CSTime::TimeStrToTime(const char* timeStr, CSTime& origtime)
{
    bool status = false;
    const int MAX_HOUR_DIFF = 3;	// return an error if the two times are grossly different
    int ghour;
    int gmin ;
    int gsec ;
    float dsec;
    long gusec;
    double nmeatime;
    if (timeStr != NULL)
    {
        // Compute the original time from the time fed in (this comes from the string capture time)
        // Then extract the day, month and year
        int oday, omonth, oyear, oyearday;
        int ohour, omin, osec;
        long int ousec;
    //	CSTime origtime(sec, usec);		// Set up a time from seconds & usecs
        origtime.TimeToInt(ohour, omin, osec, ousec, oday, omonth, oyear, oyearday);	// Extract day, month & year

        if (sscanf_s(timeStr, "%d:%d:%f", &ghour, &gmin, &dsec) == 3)
        {
                status = true;
        }
        else if (sscanf_s(timeStr, "%lf", &nmeatime) == 1)
        {
            ghour = int(nmeatime/10000.0);
            gmin = int((nmeatime - ghour*10000.0)/100.0);
            gsec = int(nmeatime - (ghour*10000.0 + gmin*100.0));
            gusec = (long int) ((nmeatime - (int)nmeatime) * 1.e6);
            status = true;
        }
        if (status)
        {
            int dayCorrect = 0;

            // Catch situations across day boundaries

            // Original late, gps early: add one day to the time
            if ((ohour > (24 - MAX_HOUR_DIFF)) && (ghour < MAX_HOUR_DIFF))
            {
                dayCorrect = 1;
            }
            // Original early, GPS late: subtract one day from the time
            else if ((ghour > (24 - MAX_HOUR_DIFF)) && (ohour < MAX_HOUR_DIFF))
            {
                dayCorrect = -1;
            }

            // Check values are sensible
            if (abs(ghour - ohour) <= MAX_HOUR_DIFF)		// reject gross differences
            {
                if (ValidateTimeInt(ghour, gmin, gsec, gusec, oday + dayCorrect, omonth, oyear))
                {
                // Construct a time object from the combination
                //CSTime gpstime(ghour, gmin, gsec, gusec, oday + dayCorrect, omonth, oyear);
                IntToTime(ghour, gmin, gsec, gusec, oday + dayCorrect, omonth, oyear);
                }
            }
        }
    }
    return status;
}

// Return a set of ints from the current time
void CSTime::TimeToInt(int& hour, int& min, int& sec, long int& usec, int& day, int& month, int& year, int& yday) const
{
    int wday;
    TimeToInt3(hour, min, sec, usec,day, month, year, yday, wday);
}

// Return a set of ints from the current time: no microseconds
void CSTime::TimeToInt1(int& hour, int& min, int& sec, int& day, int& month, int& year, int& yday) const
{
    int wday;
    TimeToInt2(hour, min, sec, day, month, year, yday, wday);
}

// Return a set of ints from the current time: no microseconds
void CSTime::TimeToInt2(int& hour, int& min, int& sec, int& day, int& month, int& year, int& yday, int& wday) const
{
    struct tm tim;			// tm structure pointer
    time_t timet = (time_t) m_time;

    //tm_p = gmtime(&timet);	// Set the tm structure.
    gmtime_s(&tim,&timet);		//set the tim structure

    hour  = tim.tm_hour;
    min   = tim.tm_min;
    sec   = tim.tm_sec;
    day   = tim.tm_mday;
    month = tim.tm_mon + 1;		// tm encodes 0-11. Normal useage is 1-12.
    year  = tim.tm_year + 1900;	// tm encodes as year - 1900
    yday  = tim.tm_yday;
    wday  = tim.tm_wday;
}

// Return a set of ints from the current time
void CSTime::TimeToInt3(int& hour, int& min, int& sec, long int& usec, int& day, int& month, int& year, int& yday, int& wday) const
{
    double dint;		// temp store for the integer part
    usec = (long int) (modf(m_time, &dint) * CSTimeSToUS);

    // Old and broken version:
    //usec = ((long int) (m_time * CSTimeSToUS)) % CSTimeSToUS; // Get the microseconds (this overflows and fails)
	
    // Call the non-microseconds version for the rest of the fields
    TimeToInt2(hour, min, sec, day, month, year, yday, wday);
}

// Return a pair of long ints from the current time
void CSTime::TimeToLInt(long int& sec, long int& usec) const
{
    sec = (long int) m_time;
    usec = ((long int) ((m_time - double(sec)) * CSTimeSToUS)) % CSTimeSToUS; // Get the microseconds
}

// Return unsigned long int and int from the current time; seconds and microseconds
void CSTime::TimeToLInt2(unsigned long int& sec, int& usec) const
{
    long int sec1, usec1;
    TimeToLInt(sec1, usec1);
    sec = (unsigned long int) sec1;
    usec = (int) usec1;
}

// Return a pair of long ints from the current time; seconds and milliseconds
void CSTime::TimeToLIntMs(long int& sec, long int& msec) const
{
    sec = (long int) m_time;
    msec = ((long int) ((m_time - double(sec)) * 1000)) % 1000;		// Get the milliseconds
}

// Return a string, set from the current time
void CSTime::TimeToStr(char *string, int strlen) const
{
    int hour, min, sec, day, month, year, yday;
    TimeToInt1(hour, min, sec, day, month, year, yday);
    sprintf_s(string, strlen, "%02d:%02d:%02d %02d/%02d/%04d", hour, min, sec, day, month, year);
}

// Return a string, set from the current time: show decimal seconds, but not date
void CSTime::TimeToStr1(char *string, int strlen) const
{
    int hour, min, sec, day, month, year, yday;
    long int usec;
    double dsec;

    TimeToInt(hour, min, sec, usec, day, month, year, yday);
    dsec = double(sec) + double(usec) * 1.e-6;
    sprintf_s(string, strlen, "%02d:%02d:%08.5f", hour, min, dsec);
}

// Return a string, set from the current time: don't show seconds
void CSTime::TimeToStr2(char *string, int strlen) const
{
    int hour, min, sec, day, month, year, yday;
    TimeToInt1(hour, min, sec, day, month, year, yday);
    sprintf_s(string, strlen, "%02d:%02d %02d/%02d/%04d", hour, min, day, month, year);
}

// Return a string, set from the current time: show decimal seconds, and date
// Needs string to be at least 25 long
void CSTime::TimeToStr3(char *string, int strlen) const
{
    int hour, min, sec, day, month, year, yday;
    long int usec;
    double dsec;

    TimeToInt(hour, min, sec, usec, day, month, year, yday);
    dsec = double(sec) + double(usec) * 1.e-6;
    sprintf_s(string, strlen, "%02d:%02d:%08.5f %02d/%02d/%04d", hour, min, dsec, day, month, year);
}

// Return a string, set from the current time - date first
void CSTime::TimeToStr4(char *string, int strlen) const
{
    int hour, min, sec, day, month, year, yday;
    TimeToInt1(hour, min, sec, day, month, year, yday);
    sprintf_s(string, strlen, "%02d/%02d/%04d %02d:%02d:%02d", day, month, year, hour, min, sec);
}

// Return an NMEA-type time string, set from the current time (HHMMSS.SSS)
// Needs string to be at least 10 long (NEMAStringLength == 12)
void CSTime::TimeToNMEAString(char *string) const
{
    int hour, min, sec, day, month, year, yday;
    long int usec;
    double dsec;

    TimeToInt(hour, min, sec, usec, day, month, year, yday);
    dsec = double(sec) + double(usec) * 1.e-6;
    sprintf_s(string, NEMAStringLength, "%02d%02d%06.3f", hour, min, dsec);
}

// Return the time in seconds as a string
void CSTime::TimeToSecondsString(char *string, int strlen) const
{
    sprintf_s(string, strlen, "%.5f", m_time);
}



// Set the time from the GPS time string. NMEA strings don't include date,
// so we have to get that from the PC clock, as fed in from the original data strings
// Passes the corrected time back by reference, and returns good or bad status

bool CSTime::TimeFromGPSTime(double nmeagpstime, CSTime& origtime)
{
    bool status = false;
    const int MAX_HOUR_DIFF = 3;	// return an error if the two times are grossly different

    // Compute the original time from the time fed in (this comes from the string capture time)
    // Then extract the day, month and year
    int oday, omonth, oyear, oyearday;
    int ohour, omin, osec;
    long int ousec;
//	CSTime origtime(sec, usec);		// Set up a time from seconds & usecs
    origtime.TimeToInt(ohour, omin, osec, ousec, oday, omonth, oyear, oyearday);	// Extract day, month & year

    int ghour = int(nmeagpstime/10000.0);
    int gmin = int((nmeagpstime - ghour*10000.0)/100.0);
    int gsec = int(nmeagpstime - (ghour*10000.0 + gmin*100.0));
    long int gusec = (long int) ((nmeagpstime - (int)nmeagpstime) * 1.e6);

    int dayCorrect = 0;

    // Catch situations across day boundaries

    // Original late, gps early: add one day to the time
    if ((ohour > (24 - MAX_HOUR_DIFF)) && (ghour < MAX_HOUR_DIFF))
    {
        dayCorrect = 1;
    }
    // Original early, GPS late: subtract one day from the time
    else if ((ghour > (24 - MAX_HOUR_DIFF)) && (ohour < MAX_HOUR_DIFF))
    {
        dayCorrect = -1;
    }

    // Check values are sensible
    if (abs(ghour - ohour) <= MAX_HOUR_DIFF)		// reject gross differences
    {
        if (ValidateTimeInt(ghour, gmin, gsec, gusec, oday + dayCorrect, omonth, oyear))
        {
            status = true;
            // Construct a time object from the combination
            //CSTime gpstime(ghour, gmin, gsec, gusec, oday + dayCorrect, omonth, oyear);
            IntToTime(ghour, gmin, gsec, gusec, oday + dayCorrect, omonth, oyear);
        }
    }

    return status;
}

// Set the time from the GPS time string and day, month and year
// Returns good or bad status
bool CSTime::TimeFromGPSTimeDate(const double nmeagpstime, const long nmeagpsdate)
{
    bool status = false;

    int ghour = int(nmeagpstime/10000.0);
    int gmin = int((nmeagpstime - ghour*10000.0)/100.0);
    int gsec = int(nmeagpstime - (ghour*10000.0 + gmin*100.0));
    long int gusec = (long int) ((nmeagpstime - (int)nmeagpstime) * 1.e6);
    int day = int(nmeagpsdate/10000);
    int month = int((nmeagpsdate- day*10000)/100);
    int year = int(nmeagpsdate- (day*10000 + month*100))+2000;

    // Check values are sensible
    if (ValidateTimeInt(ghour, gmin, gsec, gusec, day, month, year))
    {
        status = true;
        // Construct a time object from the combination
        IntToTime(ghour, gmin, gsec, gusec, day, month, year);
    }

    return status;
}

// Set the time from the GPS time and date string // Returns good or bad status
bool CSTime::TimeFromGPSTimeDMY(const double nmeagpstime, const int day, const int month, const int year)
{
    bool status = false;

    int ghour = int(nmeagpstime/10000.0);
    int gmin = int((nmeagpstime - ghour*10000.0)/100.0);
    int gsec = int(nmeagpstime - (ghour*10000.0 + gmin*100.0));
    long int gusec = (long int) ((nmeagpstime - (int)nmeagpstime) * 1.e6);

    // Check values are sensible
    if (ValidateTimeInt(ghour, gmin, gsec, gusec, day, month, year))
    {
        status = true;
        // Construct a time object from the combination
        IntToTime(ghour, gmin, gsec, gusec, day, month, year);
    }

    return status;
}


// Set the time in the object to the current PC time
void CSTime::TimeNow()
{
    // This version always limits time to 16ms boundaries
    //_timeb timeNow;
    //_ftime_s(&timeNow);
    //LIntToTime((long)timeNow.time, timeNow.millitm * 1000);
    //double timeTemp = m_time;

    // This should give better time resolution on systems that support it; i.e. later than Windows XP
    FILETIME fileTime;
    GetSystemTimeAsFileTime(&fileTime);		// Get the time as a FILETIME object (units of 100 nanoseconds in two parts)
    ULONGLONG ulongTime;					// Time as a long-long unsigned integer
    const ULONGLONG FTSecond = 10000000;	// One second in FILETIME units
    const ULONGLONG SEC_TO_UNIX_EPOCH = 11644473600LL * FTSecond;			// FILETIME times start in 1601; we need to move to Unix time, 1970
    ulongTime = (((ULONGLONG) fileTime.dwHighDateTime) << 32) + fileTime.dwLowDateTime - SEC_TO_UNIX_EPOCH;
    // Need to be careful here with the resolution that we can fit into a double ...
    ULONGLONG ul_seconds = ulongTime / FTSecond;
    double frac_seconds = double (ulongTime - (ul_seconds * FTSecond)) / FTSecond;
    m_time = (double) ul_seconds + frac_seconds;
}

// Set the PC clock
// This version does a basic check on the requested time.  It doesn't set it if
// the year is before a fixed value.
bool CSTime::SetPCClock()
{
    int hour, min, sec, day, month, year, yday, wday;
    long int usec;
    SYSTEMTIME time_SYSTEM;

    bool timeSet = false;

    // Create a SYSTEMTIME object, via a CTime one
    TimeToInt3(hour, min, sec, usec, day, month, year, yday, wday);
    time_SYSTEM.wHour = hour;
    time_SYSTEM.wMinute = min;
    time_SYSTEM.wSecond = sec;
    time_SYSTEM.wMilliseconds = (WORD)(usec/1000);
    time_SYSTEM.wDayOfWeek = wday;
    time_SYSTEM.wDay = day;
    time_SYSTEM.wMonth = month;
    time_SYSTEM.wYear = year;
//	CTime time_CT = CTime(year, month, day, hour, min, sec);
//	time_CT.GetAsSystemTime(time_SYSTEM);

    // Set the PC time from the SYSTEMTIME object
    if (year >= 1990)
    {
        if (SetSystemTime(&time_SYSTEM))
        {
            timeSet = true;
        }
        else
        {
            timeSet = false;
            DWORD errorCode = GetLastError();		// Mostly for debugging; we could write a message perhaps
            int temp = 0;							// Somewhere to put the breakpoint
        }
    }

    return timeSet;
}


// Set the time from a GPS time-of-week value, given that the current time is approximately correct
void CSTime::SetFromGPSTimeOfWeek(unsigned long GPSTimeOfWeek)
{
    int hour, min, sec, day, month, year, yday, wday;
    long int usec;
    TimeToInt3(hour, min, sec, usec, day, month, year, yday, wday);		// Get current calendar times
    IntToTime2(0, 0, 0, yday - wday, year);								// Go back to the start of the current week
    m_time += double(GPSTimeOfWeek)*0.001;
}

*/
// end of time object
// ******************


// **************************************************************************
// Time reference tool. Uses a time stamp and clock ticks to store clock time

/*
// Set a time in seconds
bool CSTimeRef::SetTime(double time)
{
    CSTime csTime(time);
    return SetTime(csTime);
}

// TODO: use Qt time tools
// Set a time as a CSTime
bool CSTimeRef::SetTime(CSTime& time)
{
    if (!m_writeLock)
    {
        m_writeLock = true;
        m_tickCountAtStamp = GetTickCount();		// Set the tick count
        m_timeStamp = time;
        m_writeLock = false;
        return true;
    }
    return false;
}


// Set the reference to the time of the PC clock
bool CSTimeRef::SetToPCTime()
{
    CSTime PCTime;
    PCTime.TimeNow();			// Get the PC time
    return SetTime(PCTime);		// And use it to set this time reference
}


// Get the time as a CSTime, which can then be accessed in various ways
double CSTimeRef::GetTimeD()
{
    // The current time is the time of the timestamp plus the elapsed time since the stamp, as clock ticks
    // GetTickCount() returns time since system start in milliseconds
    m_writeLock = true;
    double dTime = m_timeStamp.m_time + double(GetTickCount() - m_tickCountAtStamp) * 1e-3;
    m_writeLock = false;

    return dTime;
}

// Get the time as a CSTime, which can then be accessed in various ways
const CSTime CSTimeRef::GetTime()
{
    CSTime timeNow(GetTimeD());
    return timeNow;
}

*/

// End of time reference tool
// **************************************************************************



// Assorted utilities

// Show a time interval as a string.
// Note: CSTime doesn't help here, as it assumes a time since 1970
// To get the years right, the routine would have to know the actual date.
// Therefore, it uses a 365 day year.
void VarUtilities::TimeIntervalToString(char *string, int strlen, const double timeint)
{
    int years, days, hours, minutes, seconds;
    long int timesec;
    const double max_seconds = 99. * 365 * 24 * 3600;
    
    if (timeint == 0.)
    {
//         strcpy_s(string, strlen, "0 s");
        strncpy(string, "0 s", strlen);
        return;
    }
    
    if (timeint > max_seconds)
    {
//        strcpy_s(string, strlen, "** y *** d ** h ** m ** s");
        strncpy(string, "** y *** d ** h ** m ** s", strlen);
        return;
    }
    
    else
    {
        timesec = (long int)(timeint);
        seconds = timesec % 60;
        minutes = (timesec / 60) % 60;
        hours = (timesec / (60 * 60)) % 24;
        days = (timesec / (60 * 60 * 24)) % 365;
        years = timesec / (60 * 60 * 24 * 365);
        
        if ((years == 0) && (days == 0) && (hours == 0) && (minutes == 0))
        {
            snprintf(string, strlen, "%2d s", seconds);
            return;
        }
        else if ((years == 0) && (days == 0) && (hours == 0))
        {
            snprintf(string, strlen, "%2d m, %2d s", minutes, seconds);
            return;
        }
        else if ((years == 0) && (days == 0))
        {
            snprintf(string, strlen, "%2d h, %2d m, %2d s", hours, minutes, seconds);
            return;
        }
        else if (years == 0)
        {
            snprintf(string, strlen, "%3d d, %2d h, %2d m, %2d s", days, hours, minutes, seconds);
            return;
        }
        else
        {
            snprintf(string, strlen, "%2d y, %3d d, %2d h, %2d m, %2d s", years, days, hours, minutes, seconds);
            return;
        }
    }
}


// **********
// File names
// **********

/*
// Get the extension
QString CSFileName::GetExt()
{
    QString ext;

    int dotpos = m_name.ReverseFind('.');
    if (dotpos == 0)
        ext = "";
    else
        ext = m_name.Mid(dotpos+1);
    ext.MakeLower();

    return ext;
}

// Change the extension of a file path
QString CSFileName::SetExt(const QString ext)
{
    if ((!m_name.IsEmpty()) && (ext.GetLength() > 0))
    {
        int dotpos = m_name.ReverseFind('.');
        if (dotpos == 0)
            m_name += ext;
        else
        {
            QString newstring(m_name.Left(dotpos+1));
            m_name = newstring + ext;
        }
    }

    return m_name;
}
*/
