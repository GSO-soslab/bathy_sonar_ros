// Class that contains data for one ping from one transducer
// Data from this class is saved to disk and read in as raw data by Bathyswath Swath Processor

#ifndef PINGDATA_H
#define PINGDATA_H

#include <string>
#include <vector>
#include "BathyswathSonarDefs.h"
#include "BathyswathFileDefs.h"
#include "PPSProc.h"
#include "utilities.h"

using namespace std;

// Single phase, amplitude & time sample
class BathySample {
public:
    unsigned char ab;
    unsigned char ac;
    unsigned char ad;
    unsigned char txno;
    unsigned char samp0;
    unsigned char samp1;
    unsigned char anal0;
    unsigned char anal1;
    inline unsigned short getSamp() { return uint16_t(samp0); }
    inline unsigned short getAnal() { return uint16_t(anal0); }
    unsigned short getSampleNo() {return (samp0 + (256 * samp1));}
    unsigned short getAmplitude() {return (anal0 + (256 * anal1));}
    BathySample ();
};

// Store the four phases separately
class BathySample4Ph {
public:
    unsigned short m_sampNo;
    unsigned short m_phaseA;
    unsigned short m_phaseB;
    unsigned short m_phaseC;
    unsigned short m_phaseD;
    unsigned short m_amp;
    BathySample4Ph();
    void WriteBuffer(char* pBuffer);        // Copy to memory
    void ReadBuffer(const char* pBuffer);   // Read from memory
};

// Single complex and time sample
// Alternative to BathySample, using Q & I
// This is blatted straight out to the file as a memory image,
// so we unwrap the std::complex objects, just in case their memory allocation changes
struct BathySampleIQ
{
    unsigned short m_sampNo;		// Sample number
    float m_a_r;					// Stave A, real
    float m_a_i;					// Stave A, imag
    float m_b_r;					// Stave B, real
    float m_b_i;					// Stave B, imag
    float m_c_r;					// Stave C, real
    float m_c_i;					// Stave C, imag
    float m_d_r;					// Stave D, real
    float m_d_i;					// Stave D, imag

    BathySampleIQ();
    void WriteBuffer(char* pBuffer);        // Copy to memory
    void ReadBuffer(const char* pBuffer);	// Read from memory
};

class UsbTemSample {
public:
    unsigned char anal1;			// MS byte amplitude
    unsigned char anal0;			// LS byte, 12 bit
    unsigned char ab;				// Phase a to b
    unsigned char ac;				// Phase a to c
    unsigned char ad;				// Phase a to d
};

// Processed data samples
class ProcSample {
public:
    ProcSample();

    // polar coordinates at transducer
    double m_range;					// distance to point - polar in metres
    double m_angle_s;				// sine of angle to point - polar range -1 to +1
    double m_angle_c;				// cosine of angle to point - polar range -1 to +1

    // Cartesian coordinates of a point detected by the sonar
    C3dPos m_posV;                  // Vessel coordinates: x = forward, y = starboard, z = down
    C3dPos m_posG;                  // Geographic coordinates: x = north, y = east, z = down
    double m_hrange_ping;           // Horizontal range in the ping direction.

    unsigned short int m_amp;		// amplitude value
    unsigned short int m_procAmp;	// processed amplitude i.e. TVG value

    void setXYZZero();              // Set values to zero

    bool isOK() {return m_valid;}
    void reject() {m_valid = false;}
    bool m_valid;                   // filter status
};

// This is the header for pings sent to and received from the sonar hardware
// The parameters in the header are used to set up the sonar device, and the sonar device port fills in the parameters that it can
class PingHeader
{
public:
    PingHeader();
    ~PingHeader();

    // Ping operation
    enum  ePingOperation
    {
        PO_NONE,				// as said
        PO_SINGLE,				// single transducer
        PO_ALTERNATING,			// 2 TEMS used, 1 at a time
        PO_SIMULTANEOUS			// 2 TEMS used simultaneously
    };

    // Type of sample encoded in the data
    enum eSampleType
    {
        eFormatPhaseDiff,		// AB, AC, AD phase differences storedrtc as chars, produced by USB systems
        eFormat4Phase,			// A, B, C, D stored as short int, created from V2
        eFormatIQ				// Raw complex float data, created by V2
    };

    void Set(bool acal, bool on, bool doPing, unsigned int cycles, unsigned int power, float gain,
             double maxPingRate, unsigned int noSamples, double period, unsigned int adcEnable, bool pps_enable,
             bool pps_rising, ePingOperation pingOp, bool usePCTime, bool externalTriggerOutPolarity,
             bool externalTriggerEdge, bool externalTrigger, eSampleType sampleType);

    void SetPingMode(ePingOperation pingOp);

    // Write the ping header to a buffer in memory. Returns the location of the next
    // point in the buffer after the header has been written
    char* WriteBuffer(char* pBuffer, unsigned int nProcSamples, double SV = DEFAULT_SV);
    void ReadBuffer(const char * pBuffer, double& SV, unsigned int& nProcSamples, eQualityOptions& qualOption);		// Read the buffer (passes reference to pointer)
    unsigned int Size(unsigned int version);
    void Preset();
    void TextLog(std::string& textlog);
    void EnableRx();                                // Start sonar data collection
    void DisableRx();                               // Stop sonar collection
    bool RxIsEnabled();
    void EnableTx();                                // Turn on sonar transmitter
    void DisableTx();                               // Turn off sonar transmitter
    bool TxIsEnabled();
    void SetVersion(unsigned int aVer);             // SONAR_DATA or SONAR_DATA2 or SONAR_DATA3 legal
//    double GetNominalRange() const;					// Get the slant range set
//    void SetNominalRange(double range);             // Set the transmit parameters, given a nominal slant range in metres
//    double GetSampNominalRange(int iSamp) const;    // Get the slant range for a sample, using the nominal SV
//    double GetSampRange(int iSamp, double SV) const; // Get the slant range for a sample, using real SV
//    unsigned int GetSampForRange(double range) const;  // Get the sample number for a range, using nominal SV
    double GetPingTimePC() { return double(m_timeSecPC) + double(m_timeMsecPC) / 1000.0; }
    unsigned int GetVersion() const {return m_version;}

    CPPSProc m_PPSProc;                             // class for setting & interpreting the PPS bitfield
    void SetPPSProc() {m_PPSProc.m_pPPS = &m_PPS;}	// (re)set the pointer to the m_PPS item

    // ***************************************
    // User settings for controlling the sonar

    unsigned int m_pingNum;			// Unique ping number
    unsigned int m_tdrChannel;		// Identifies the transducer number, generally 1, 2 or 3 (1-based)
    unsigned int m_stavesInTem;		// Number of staves in a TEM. Currently always 4, but could change
    unsigned int m_fpgaIdent;		// FPGA code identifier.
    // Bytes are fpgaCompilerMinor | fpgaCompilerMajor | fpgaVerMinor | fpgaVerMajor
    // From major ver 7 and up.
    unsigned int m_tdrType;         // Transducer code, read from the transducer connector pins.
    // TXD_TYPE_117 etc.
    unsigned int m_boardType;       // Board Type code, set in FPGA. BRD_TYPE_117 etc.
    long long m_boardIdent;			// Serial number, unique to each board.
    float	m_operatingFreq;		// Sonar frequency of transducer, in Hz, interpreted from FPGA registers.
    float	m_analogueGain;			// Currently not used; set to one in the system but otherwise ignored
    int		m_noClocksIn360;		// Number of phase bits for 2PI of phase
    // Generally 256 for an 8-bit phase system, but could be different in future.
    // Encoded and read from the TEM FPGA
    unsigned int m_FPGAerror;		// Error flags read from the TEM FPGA
    bool	m_calBit;               // Control bits- cal
    bool	m_pingBit;              // Sonar transmitter on/off - Write only to FPGA; not cleared by FPGA
    bool	m_rxBit;                // Receive data on/off - always set in Usb write operation
    bool	m_resetBit;             // Write only to FPGA; not cleared by FPGA
    unsigned int m_txPower;         // Transmit power
    unsigned int m_txCycles;        // Length of the transmit pulse, in sonar cycles
    unsigned int m_rxSamples;       // Number of samples held / to be held in the data buffer
    float	m_rxPeriod;             // Period between received samples, in seconds
    double	m_maxPingRate;			// Maximum ping rate allowed
    unsigned int m_sidescanAdcEnable; // Determines which transducer stave(s) are used for amplitude
    // Called Channel in FPGA docs
    int		m_doTimeUpdate;			// Flag: update the time in the FPGA.
    // 1 = command a time update from Tx reg set; 0 = ignore
    unsigned int m_timeSecPC;       // Ping time, seconds. Lasts until the year 2039.
    unsigned int m_timeMsecPC;		// Ping time, mS
    unsigned int m_timeSecSonar;	// Ping time, seconds
    unsigned int m_timeMsecSonar;	// Ping time, mS
    unsigned int m_firstInScan;		// Used for alternating patterns of pings
    // first = always 1; alternating or double-sided goes 1,0,1,0..
    unsigned char	m_PPS;			// bitfield representing 1PPS setting & status
    unsigned int m_pingMode;        // Enum indicating mode (single/alt/sim), tx on/off, and port/stbd

    bool m_externalTrigger;			// Use external triggering
    bool m_externalTriggerOutPolarity;
    bool m_externalTriggerEdge;

    // V2 settings
    unsigned short	m_triggerFlags;	//  Trigger in and out flags. 0: no trigger
    unsigned short 	m_pgagain;		//	Gain in the Programmable Gain Attenuators in 1/100th db (translated from the CV2Ping when the ping is set up from the V2 data)
    unsigned short 	m_lnagain;		//	Gain in the Low Noise Amplifiers in 1/100th db
    unsigned short 	m_basegain;		//	Gain in 1/100th db at the end of the transmit pulse
    unsigned short 	m_lingain;		//	Amount in 1/100th db/ms the gain rises linearly
    unsigned short 	m_sqgain;		//	Amount in 1/100th db/ms^2 the gain rises as the square of time
    float		m_swgain;           //	Final gain in dB that is applied in software
    unsigned short	m_RxDecimation;	//	Hardware sends 1 in n samples; 1 = no decimation
    float		m_rxBandwidth;		//	Filter bandwidth in Hz
    bool		m_preampPowerOn;	//	0 = power off, 1 = on
    eSampleType		m_sampleType;   // The sample type used in this ping

    // End of user settings
    // ********************

    bool            m_gapsInData;	// Flag: some data blocks have been dropped, so the data isn't contiguous
    unsigned char   m_ampError;

    // Sonar mode & status byte
    enum ePingMode
    {
        SONAR_SEL_OFF,		//	Not used
        SONAR_SEL_SINGLE,	//	Single-sided pinging
        SONAR_SEL_ALT,		//	Alternating pinging
        SONAR_SEL_SIM		//	Simultaneous pinging
    };

    enum eStatusMode
    {
        SONAR_STATUS_MODE       = 0x3,		// Bit mask for mode bits (ePingMode)
        SONAR_STATUS_TX_BIT     = 0x4,		// Tx on (otherwise receive only)
        SONAR_STATUS_PORT_BIT	= 0x8		// Port-starboard (set 1 for stbd)
    };

private:
    int lastTdrChannel;                     // Signed, because uses negative to indicate not assigned yet
    unsigned int m_version;                 // SONAR_DATA or SONAR_DATA2 or SONAR_DATA3
};

class PingData
{
public:
    PingData();
    ~PingData();

    void WriteBuffer(char* pBuffer, unsigned int bufferSize, unsigned int version);     // Build a byte array for serialization to disk
    bool ReadBuffer(const char *pBuffer, unsigned int aVer, unsigned int length);       // Deserialize from a byte array (reading from disk)

    void SetSamples(unsigned int nSamps) {m_header.m_rxSamples = nSamps; CreateArray();}	// Set the number of samples and check that the array is big enough
    void ClearSamples();                                            // Empty out the sample data
    void Copy(PingData* other);                                     // Copy another ping to this one
    void CopyArray(PingData* other);                                // Copy data from another array
    void NullArrays();                                              // Set array pointers to null without deleting them; neceassary after a shallow copy, so that the copied instatiation deletes them
    unsigned int GetMeanAmplitude();                                // Get the mean amplitude from the array
    unsigned int GetMeanAmplitude(double& sd);                      // Get the mean amplitude and standard deviation from the array
    void EnableTx();                                                // Turn on sonar transmitter
    unsigned int Size(unsigned int version);                        // Get the buffer size needed to serialize this ping using a particular file version
    void setVersion(unsigned int version);                          // Set the data format version number
    std::string getDataSummary();                                   // Return text summarising the data

    bool isEmpty() {return (m_header.m_rxSamples <= 0);}            // Are there any samples?
    unsigned int GetSampleCount() const {return m_header.m_rxSamples;}
    unsigned int GetProcessedCount() const {return unsigned (m_procSamples.size());}
    double GetNominalFrequency() const;
    double GetNominalRange() const;                                 // Get the slant range set
    void SetNominalRange(double range);                             // Set the transmit parameters, given a nominal slant range in metres
    double GetSampNominalRange(int iSamp) const;                    // Get the slant range for a sample, using the nominal SV
    double GetSampRange(int iSamp, double SV) const;                // Get the slant range for a sample, using real SV
    unsigned int GetSampForRange(double range) const;               // Get the sample number for a range, using nominal SV
    unsigned short GetSampAmplitude(unsigned int iSamp) const;      // Get the amplitude for this sample
    bool GetSampPhases(int iSamp, unsigned char& ab, unsigned char& ac, unsigned char& ad) const;     // Get phase data for one sample
    bool GetMinMaxAmplitude(double& minAmp, double& maxAmp, double minRange = 0.) const;        // Get minimum and maximum amplitude for this ping
    double getTimeOffset() const;                                   // Get the fixed time offset for data samples given the version of the sonar and transmit length

    void SetTriggerValues(bool extTrigger, unsigned int extTriggerSource, bool extTriggerInvert,
                          bool triggerOut, unsigned int triggerOutSource, bool triggerOutInvert);   // Set up the trigger code

    // Bathy processing
    void addProcSample(ProcSample procSample);

    std::string getSummaryString();                                 // A string summarising the header

    PingHeader	m_header;                                           // m_header.m_rxSamples gives count of samples loaded
    BathySample* m_samples;                                         // eFormatPhaseDiff:	AB, AC, AD phase differences stored as chars, produced by USB systems
    BathySample4Ph* m_4phSamples;                                   // eFormat4Phase:		A, B, C, D stored as short int, created from V2
    BathySampleIQ* m_IQsamples;                                     // eFormatIQ:			Raw complex float data, created by V2
    unsigned int m_arraySize;                                       // Size of the data buffer
    PingHeader::eSampleType	m_arrayType;                            // Data type of the current array; 0: phase diff format, 1: 4-phase format, 2: IQ format. We need to create a new array if m_sampleType is different

    vector<ProcSample> m_procSamples;                               // Processed sonar data

private:
    bool CreateArray();                                             // Make an array for the samples
    bool CreateProcArray(unsigned int nProcSamples);                // Set up the processed data array
    void DeleteArrays();                                            // Delete any existing data arrays
};

#endif // PINGDATA_H
