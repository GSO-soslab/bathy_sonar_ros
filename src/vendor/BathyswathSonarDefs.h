// Generic definitions for the Bathyswath V2 sonar

#ifndef BATHYSWATHSONARDEFS_H
#define BATHYSWATHSONARDEFS_H

namespace BSWSnr {

// Generic sonar parameters
const int	MAXSAMPS		= 0xff00;		// Samples per ping
const int	MAXCHANS		= 4;			// Initial version limited to FOUR transducer channels
const int	NUM_TDCRS		= 3;			// Number of Bathyswath transducers supported. The fourth channel is used for MBES; handle this if we add MBES handling to SwathRT.
const int   SNR_CHANS       = MAXCHANS * NUM_TDCRS;     // Total number of sonar channels
const int	MAXBUFS			= 12;			// Number of buffers in linked-list structure
const int	MAX_TX			= 4;			// Maximum number of TEM channels
const int	MBES_CHAN		= 4;			// Put MBES data into channel 4
const int	MAX_TX_SLOTS	= MAX_TX + 1;	// TEM channel numbers are 1-based, so this is the array size we need. Historical note: in the ISA system, channel zero as a setting meant "alternate"
const int   V2_STAVES_PER_CHANNEL = 4;      // Number of staves in each channel in a V2 system
const double V2_SAMPLE_TIME_OFFSET = -213.5e-6;

// Transducer Frequencies
// These are binary divisions of 30 MHz
const double TXD_FREQ_029 = 29.296875e3;
const double TXD_FREQ_039 = 39.0625e3;
const double TXD_FREQ_040 = 40.7609e3;
const double TXD_FREQ_058 = 58.59375e3;
const double TXD_FREQ_117 = 117.1875e3;
const double TXD_FREQ_234 = 234.375e3;
const double TXD_FREQ_468 = 468.750e3;

// const double DEFAULT_SV = 1490.;               // default speed of sound

// Board type Identifiers
const unsigned char BRD_TYPE_117_Q0		= 1;	// Quicklogic based 64 way DIN41612 interface (117kHz only) Development only: shouldn't find in the field
const unsigned char BRD_TYPE_117		= 2;	// Quicklogic based 37 way D connector interface 117KHz
const unsigned char BRD_TYPE_ISA		= 3;	// 16-Bit ISA Board
const unsigned char BRD_TYPE_234		= 4;	// Quicklogic based 37 way D connector interface 234KHz
const unsigned char BRD_TYPE_117_A		= 5;	// Altera based 37 way D connector interface 117KHz
const unsigned char BRD_TYPE_234_A		= 6;	// Altera based 37 way D connector interface 234KHz
const unsigned char BRD_TYPE_468_A		= 7;	// Altera based 37 way D connector interface 468KHz
const unsigned char BRD_TYPE_USB		= 8;	// Altera based, USB interface TEM, all frequencies (freq. in a different register)
const unsigned char BRD_TYPE_MK4		= 9;	// Altera based, USB interface TEM, initially for TOBI (freq. in a different register)
const unsigned char BRD_TYPE_V2_A		= 10;	// First version of the V2 TEMs

// Transducer type Identifiers
// TODO: merge these with enum eTransducerType
const unsigned char TXD_TYPE_NO_CONN	= 15;	// No transducer connected to TEM
const unsigned char TXD_TYPE_117		= 10;	// SWATHplus/Bathyswath 1 117kHz
const unsigned char TXD_TYPE_234		= 5;	// SWATHplus/Bathyswath 1 234kHz
const unsigned char TXD_TYPE_468		= 13;	// SWATHplus/Bathyswath 1 468kHz
const unsigned char TXD_TYPE_117_V2		= 16;	//           Bathyswath 2 117kHz
const unsigned char TXD_TYPE_234_V2		= 17;	//           Bathyswath 2 234kHz
const unsigned char TXD_TYPE_468_V2		= 18;	//           Bathyswath 2 468kHz
const unsigned char TXD_TYPE_SD_01      = 19;	// Seal detector system type 1

// For sanity checking: 20pps for a week
const unsigned long int MAX_PING_NUM = 12096000;

// Type of TEM
enum e_PortType
{
    TEM_NONE,		// Not identified
    TEM_USB,		// USB TEMs
    TEM_V2,			// Bathyswath V2 TEMs
    TEM_ISA,		// ISA TEMs (we might be dropping support for these now)
};

// Ping operation
enum  ePingOperation
{
    PO_NONE,				// as said
    PO_SINGLE,				// single transducer
    PO_ALTERNATING,			// 2 TEMS used, 1 at a time
    PO_SIMULTANEOUS			// 2 TEMS used simultaneously
};

enum eTEMPort {eRS485_1, eRS485_2, eGPIO_1, eGPIO_2,};          // Port on the TEM used for PPS or triggers

enum eTransducerType					// Transducer types
{
    eTransducerType_None        = 15,				// None
    eTransducerType_Default     = 0,                // Default
    eTransducerType_468V1       = 13,				// 468 kHz V1       V1 types do not have preamplifiers (some vehicles might have transducers fitted without preamps to save space and power)
    eTransducerType_234V1       = 5,				// 234 kHz V1
    eTransducerType_117V1       = 10,				// 117 kHz V1
    eTransducerType_468V2       = 18,				// 468 kHz V2       V2 types have preamplifiers
    eTransducerType_234V2       = 17,				// 234 kHz V2
    eTransducerType_117V2       = 16,				// 117 kHz V2
    eTransducerType_SD500_01    = 19,               // 500kHz SD transducer type 01
};

}

#endif // BATHYSWATHSONARDEFS_H
