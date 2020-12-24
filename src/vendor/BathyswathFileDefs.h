// Generic definitions for Bathyswath data files

#ifndef BATHYSWATHFILEDEFS_H
#define BATHYSWATHFILEDEFS_H

// General data file parameters
// TODO: consider moving this to a separate .h file, if we start handling other kinds of "sbx" data blocks, e.g. attitude data

// ****************************
// Data type number definitions

// *************
// Block headers
const int FIRST_SXF_TYPE		= 0x00;		// First type in the list
const int SONAR_DATA			= 0x00;		// Deprecated. Use SONAR_DATA4
const int COMPASS_DATA			= 0x01;
const int MRU_DATA				= 0x02;
const int GPS_DATA				= 0x03;
const int HWARE_DATA			= 0x04;
const int FILE_DATA				= 0x05;
const int GUI_DATA				= 0x06;
const int NET_DATA				= 0x07;
const int COMPASST_DATA			= 0x08;
const int MRUT_DATA				= 0x09;
const int GPST_DATA				= 0x0a;
const int AUX1T_DATA			= 0x0c;
const int PHCAL_DATA			= 0x0d;
const int COVRG_DATA			= 0x0e;
const int AUX2T_DATA			= 0x10;
const int TEXT_DATA				= 0x11;		// Text message
const int SYSTEM_COMMAND_DATA   = 0x12;		// Command from one system to another
const int TIME_SYNCH_DATA       = 0x13;		// Time synchronisation between systems
const int SONAR_COMMAND_DATA    = 0x14;		// Command sent to set up the sonar on the remote system
const int SONAR_DATA2_TEST		= 0x15;		// replacement for SONAR_DATA; an incorrect version that appeared briefly in testing
const int SONAR_DATA2			= 0x16;		// Deprecated. Use SONAR_DATA3
const int SONAR_DATA3			= 0x17;		// replacement for SONAR_DATA & SONAR_DATA2.
const int SONAR_DATA4A			= 0x18;		// Raw file format introduced to support Bathyswath V2, Nov 2014; prototype, missing some parameters
const int SONAR_DATA4			= 0x19;		// Raw file format introduced to support Bathyswath V2, Feb 2015
const int SEAL_DATA             = 0x1a;     // Raw file format for seal detector systems
const int SD_MOTION_DATA        = 0x1b;     // Motion data from seal detector systems
const int SEAL_DATA2            = 0x1c;     // Raw file format for seal detector systems, version 2


// File header identifiers for configuration data.
const int CNF_SENSOR_CORR 		= 0x20; 	// Sensor corrections
const int CNF_SENSOR_FILT		= 0x21; 	// Sensor filters
const int CNF_SENSOR_INTERP		= 0x22; 	// Sensor interpolation
const int CNF_DERIVE_ATT  		= 0x23; 	// Attitude derivation
const int CNF_ENVIR				= 0x24; 	// Environment parameters
const int CNF_DERIVE_POSN		= 0x25; 	// Position derivation
const int CNF_TOW_POSN			= 0x26; 	// Tow offsets: towed vehicles
const int CNF_POSN_OFFSETS		= 0x27; 	// Offsets between sensors

// Processed data file (SXP)
const int SBP_XYZA_PING			= 0x28; 	// Deprecated (use SBP_XYZA_PING2)
const int SBP_XYZA_PING2		= 0x52;     // Ping data for processed data xyza: auxiliary data
const int SBP_PROJECTION		= 0x50;		// Information on the projection used in processing (PLACEHOLDER AT 25/07/07 MFG)
const int SBP_PROCESS_INFO		= 0x51;		// Information on the processes used in processing (PLACEHOLDER AT 25/07/07 MFG)

// Obsolete Submetrix raw data file format
const int SBR_RAW_PING			= 0x29; 	// Ping data for raw data (obsolete, luckily, as this number got re-used)

// Parsed data items, used in "SXI" files
const int PARSED_PING_DATA		= 0x29;		// Sonar data in parsed data
const int PARSED_ATTITUDE		= 0x2b;		// Attitude data in parsed data
const int PARSED_POSITION_LL	= 0x2c;		// Lat-long position data in parsed data
const int PARSED_POSITION_EN	= 0x2d;		// Easting-Northing position in parsed data
const int PARSED_SVP			= 0x2e;		// Speed of sound data in parsed data
const int PARSED_ECHOSOUNDER	= 0x2f;		// Echosounder data in parsed data
const int PARSED_TIDE			= 0x30;		// Tide data in parsed data
const int PARSED_AGDS			= 0x31;		// AGDS data in parsed data
const int PARSED_AUX_STR        = 0x32;     // Auxiliary string in 'parsed' data
const int PARSED_POSITION_LL_2  = 0x33;     // Lat-long position data in parsed data, plus altitude

const int CMS_CMD				= 0x40;		// Commands sent to Swath by external processes
const int CMS_STATUS			= 0x41;		// Status summary sent by Swath to external processes
const int AUX_ATTPOS			= 0x42;		// Attitude & position data sent in the BAUUV IST interface

const int AUX3T_DATA			= 0x61;
const int AUX4T_DATA			= 0x62;
const int AUX5T_DATA			= 0x63;
const int AUX6T_DATA			= 0x64;
const int AUX7T_DATA			= 0x65;

const int MBES_DATA				= 0x66;		// Wrapper for PICOSonar data

// Last type in the list of standard Bathyswath blocks (* UPDATE WHEN ADDING NEW ITEMS. PLEASE *)
const int LAST_SXF_TYPE			= MBES_DATA;

const int DUMMY_SXF_DATA_TYPE	= 0xFFFF;	// A dummy type, e.g. used to force a channel without having a payload

// PicoSonar data objects; similar to Bathyswath data blocks, so some common processing is possible
const int PICOMBES_DATA = 0X51C03BE5;		// Header ID for PicoMBES & Pico3D data. These items don't follow exactly the block format of other blocks, but at least the first 4 bytes are the block ID.
const int PICODPCA_DATA = 0X51C0D5CA;		// DPCA micro-nav header ID

const int SONAR_DATA2_TEST_HEAD_SIZE = 40;	// Size of header
const int SONAR_DATA2_HEAD_SIZE = 41;		// Size of header
const int SONAR_DATA3_HEAD_SIZE = 49;		// Size of header
const int SONAR_DATA4A_HEAD_SIZE = 48;		// Size of header for initial version of SONAR_DATA4
const int SONAR_DATA4_HEAD_SIZE = 71;		// Size of header
const int SONAR_DATA_HEAD_SIZE = 72;		// Size of header (old style)
const int BathySample4Ph_SIZE = 6 * sizeof(unsigned short);							// Size of BathySample4Ph when copied to memory
const int BathySampleIQ_SIZE = sizeof(unsigned short) + (8 * sizeof(float));		// Size of eFormatIQ when copied to memory

const int PARSED_PING_DATA_HEADER_SIZE = 35;     // Size of the header section in the PARSED_PING_DATA format
const int PARSED_PING_DATA_ITEM_SIZE = 7;        // Size of each data item in the PARSED_PING_DATA format

const int FILE_FORMAT_MAJOR_VERSION = 1;
const int FILE_FORMAT_MINOR_VERSION = 0;
const int FILE_SOFTWARE_MAJOR_VERSION = 1;
const int FILE_SOFTWARE_MINOR_VERSION = 0;
const int SXF_HEADER_FILE_FORMAT_LENGTH = 4;
const int SXF_HEADER_SOFTWARE_LENGTH = 4;
const int SXF_BLOCK_CODE_LENGTH = 4;
const int SXF_BLOCK_LENGTH_LENGTH = 4;
const int SXF_HEADER_BLOCK_LENGTH = SXF_BLOCK_CODE_LENGTH + SXF_BLOCK_LENGTH_LENGTH;

const int PARSED_ATTITUDE_SIZE		= 25;
const int PARSED_POSITION_LL_SIZE	= 25;
const int PARSED_POSITION_LL2_SIZE	= 41;
const int PARSED_POSITION_EN_SIZE	= 25;
const int PARSED_SVP_SIZE		= 13;
const int PARSED_ECHOSOUNDER_SIZE	= 13;
const int PARSED_TIDE_SIZE		= 13;
const int PARSED_AGDS_SIZE		= 17;

// Data quality options used in PARSED_PING_DATA
enum eQualityOptions
{
    QUALITY_MERGED,					// Quality byte is a merged set of phase error, amplitude and range
    QUALITY_PHASE_ERROR,				// Just the phase error
    QUALITY_FILTER_FLAGS				// A set of 8 bitwise flags, giving the result of up to 8 filters
};

// Sonar mode & status byte
enum ePingMode
{
    SONAR_SEL_OFF,		//	Not used
    SONAR_SEL_SINGLE,	//	Single-sided pinging
    SONAR_SEL_ALT,		//	Alternating pinging
    SONAR_SEL_SIM		//	Simultaneous pinging
};
const int PARSED_RESERVED_BYTES		= 2;
const double PARSED_ANGLE_SCALE = 32768 / 3.1415926535897932384626433832795;	// Angle scaled as radians vs 16-bit number

// *************************
// File header magic numbers
const int SXR_HEADER_DATA	= 0xbad0bad0;           // raw data
const int SXC_HEADER_DATA   = 0xf1c0f1c0;           // configuration file
const int SXP_HEADER_DATA   = 0x01df01df;           // processed data file
const int SXG_HEADER_DATA   = 0xd1edede0;           // grid file
const int SWC_HEADER_DATA   = 0xc311c311;           // Coverage data file
const int SXI_HEADER_DATA   = 0x521d52d1;           // parsed data file
const unsigned char XTF_HEADER_DATA   = 0x7b;       // xtf data file
const unsigned short XTF_MAGIC_NUMBER   = 0xface;   // xtf data file

// End of Data type number definitions
// ***********************************

// *************
// File versions
// These are effectively multiplied by 100.
// i.e. version 1.00 is coded as 100, version 1.23 is coded as 123, etc.
// Current version numbers: update if the file format in this file changes

// Raw data file
const int Sxr_sw_version = 100;
const int Sxr_fmt_version = 100;

// Processed data file
const int Sxp_sw_version = 100;
const int Sxp_fmt_version = 101;

// Configuration data file
const int Cnf_sw_version = 100;
const int Cnf_fmt_version = 101;

// Coverage data file
const int Swc_sw_version = 100;
const int Swc_fmt_version = 100;

// Parsed data file
const int Sxi_sw_version = 100;
const int Sxi_fmt_version = 100;

// XTF data file
const int Xtf_sw_version = 326;
const int Xtf_fmt_version = 121;

// End of file verions
// *******************

// File data block sizes
const int SXF_MAXSAMPS = 0xff00;  // Samples per ping
const int SXF_MAXCHANS = 4;       // Initial version limited to FOUR transducer channels
const int SXF_MAXBLOCKSIZE = SXF_MAXSAMPS * SXF_MAXCHANS * 8;	// Max. size of a data block
const int SXF_MIN_BLOCKSIZE = 4;    // Smalllest data payload we expect

const int SXP_OLD_TXER_SIZE = 128;
const int SXP_OLD_POINT_SIZE = 40;

#endif // BATHYSWATHFILEDEFS_H
