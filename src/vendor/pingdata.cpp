// Class that contains data for one ping from one transducer
// Data from this class is saved to disk and read in as raw data by Bathyswath Swath Processor
#include "pingdata.h"
#include "HardwareInterfaceV2.h"
#include <QtDebug>
#include <cstring>
#include <iostream>
#include <cmath>
#include <limits>
#include "utilities.h"

using namespace std;
using namespace BSWSnr;
using namespace utilities;

PingData::PingData()
{
    m_header.Preset();
    m_samples = nullptr;
    m_4phSamples = nullptr;
    m_IQsamples = nullptr;
    m_arraySize = 0;
    m_arrayType = PingHeader::eFormatPhaseDiff;
}

PingData::~PingData()
{
    DeleteArrays();
}

// Delete any existing data arrays
void PingData::DeleteArrays()
{
    if (m_samples != nullptr)
    {
        delete[] m_samples;
        m_samples = nullptr;
    }
    if (m_4phSamples != nullptr)
    {
        delete[] m_4phSamples;
        m_4phSamples = nullptr;
    }
    if (m_IQsamples != nullptr)
    {
        delete[] m_IQsamples;
        m_IQsamples = nullptr;
    }
    m_arraySize = 0;
}

// Set array pointers to null without deleting them; neceassary after a shallow copy, so that the copied instatiation deletes them
void PingData::NullArrays()
{
    m_samples = nullptr;
    m_4phSamples = nullptr;
    m_IQsamples = nullptr;
}


// Make a raw data array, of whichever type is needed, if necessary
bool PingData::CreateArray()
{
    if ((m_header.m_rxSamples > 0) && (m_header.m_rxSamples < MAXSAMPS))			// Sanity check
    {
        // We need to make a new array if either:
        // - the array of the type that we need hasn't been created yet, or
        // - we need a bigger array
        // Otherwise, we leave it alone

        bool buffNull = false;
        switch (m_header.m_sampleType)
        {
        case PingHeader::eFormatPhaseDiff:	buffNull = (m_samples == nullptr); break;
        case PingHeader::eFormat4Phase:		buffNull = (m_4phSamples == nullptr); break;
        case PingHeader::eFormatIQ:			buffNull = (m_IQsamples == nullptr); break;
        }

        if (buffNull || (m_arraySize <  m_header.m_rxSamples) || (m_header.m_sampleType != m_arrayType))
        {
            DeleteArrays();
            const double slop = 0.2;		// Allow some slop for variation in array sizes
            m_arraySize = uint32_t(m_header.m_rxSamples * (1 + slop));
            // Create an array of the type needed, the () should initialise to zero
            switch (m_header.m_sampleType)
            {
            case PingHeader::eFormatPhaseDiff:	m_samples = new BathySample[m_arraySize](); break;
            case PingHeader::eFormat4Phase:		m_4phSamples = new BathySample4Ph[m_arraySize](); break;
            case PingHeader::eFormatIQ:			m_IQsamples = new BathySampleIQ[m_arraySize](); break;
            }
            m_arrayType = m_header.m_sampleType;
        }
        return true;
    }
    return false;
}

// Set up the processed data array
bool PingData::CreateProcArray(unsigned int nProcSamples)
{
    if (nProcSamples < SXF_MAXSAMPS)
    {
        m_procSamples.resize(nProcSamples);
        return true;
    }
    return false;
}

// Copy another ping to this one
void PingData::Copy(PingData* other)
{
    m_header = other->m_header;			// Use the default copy method for the header
    m_header.SetPPSProc();				// (re)set the pointer to the m_PPS item

    CopyArray(other);					// Copy the data
}

// Copy data from another array
void PingData::CopyArray(PingData* other)
{
    m_header.m_rxSamples = other->m_header.m_rxSamples;
    m_header.m_sampleType = other->m_header.m_sampleType;
    CreateArray();
    switch (m_header.m_sampleType)
    {
    case PingHeader::eFormatPhaseDiff:	if (other->m_samples != nullptr)	memcpy(m_samples, other->m_samples, m_header.m_rxSamples * sizeof(BathySample)); break;
    case PingHeader::eFormat4Phase:		if (other->m_4phSamples != nullptr)	memcpy(m_4phSamples, other->m_4phSamples, m_header.m_rxSamples * sizeof(BathySample4Ph)); break;
    case PingHeader::eFormatIQ:			if (other->m_IQsamples != nullptr)	memcpy(m_IQsamples, other->m_IQsamples, m_header.m_rxSamples * sizeof(BathySampleIQ)); break;
    }
}

// Get the mean amplitude from the array
unsigned int PingData::GetMeanAmplitude()
{
    double sd;
    return GetMeanAmplitude(sd);
}

// Get the mean amplitude and standard deviation from the array
unsigned int PingData::GetMeanAmplitude(double& sd)
{
    double totalAmplitude = 0;
    double totalAmplitudeSq = 0;
    int count = 0;
    unsigned int firstPoint = uint32_t(m_arraySize * 0.2);		// Skip the first part, to avoid the transmit pulse

    for (unsigned int i = firstPoint; i < m_header.m_rxSamples; i++)
    {
        switch (m_header.m_sampleType)
        {
        case PingHeader::eFormatPhaseDiff:
            if (m_samples)
            {
                unsigned short amplitude = uint16_t(m_samples[i].anal0 + (uint32_t(m_samples[i].anal1) << 8));
                totalAmplitude += amplitude;
                totalAmplitudeSq += double(amplitude) * amplitude;
                count++;
            }
            break;
        case PingHeader::eFormat4Phase:
            if (m_4phSamples)
            {
                totalAmplitude += m_4phSamples[i].m_amp;
                count++;
            }
            break;
        case PingHeader::eFormatIQ:
            //	TO DO
            break;
        }
    }

    unsigned int meanAmplitude = 0;
    sd = 0;
    if (count > 0)
    {
        meanAmplitude = uint32_t(totalAmplitude / count);
        // the square root of the difference between the average of the squares of the values and the square of the average value.
        sd = sqrt(fabs((totalAmplitudeSq / count) - ((double(meanAmplitude) * meanAmplitude))));
    }

    return meanAmplitude;
}

//-------------------------------------------------
// We are unlikely to see data types earlier than SONAR_DATA4
bool PingData::ReadBuffer(const char *pBuffer, unsigned int aVer, unsigned int length)
{
    bool status = true;

    uint32_t samplesOffset;
    m_header.SetVersion(aVer);
    double SV = 0.;
    unsigned int nProcSamples = 0;
    eQualityOptions qualOption = QUALITY_FILTER_FLAGS;
    m_header.ReadBuffer(pBuffer, SV, nProcSamples, qualOption);

    switch (aVer)
    {
    case SONAR_DATA:		samplesOffset = 0xA0;                               break;
    case SONAR_DATA2_TEST:	samplesOffset = SONAR_DATA2_TEST_HEAD_SIZE;         break;
    case SONAR_DATA2:		samplesOffset = SONAR_DATA2_HEAD_SIZE;              break;
    case SONAR_DATA3:		samplesOffset = SONAR_DATA3_HEAD_SIZE;              break;
    case SONAR_DATA4A:		samplesOffset = SONAR_DATA4A_HEAD_SIZE;             break;
    case SONAR_DATA4:		samplesOffset = SONAR_DATA4_HEAD_SIZE;              break;
    case PARSED_PING_DATA:	samplesOffset = PARSED_PING_DATA_HEADER_SIZE; 		break;

    default: return false;
    }

    if (aVer == PARSED_PING_DATA)
    {
        if (length < (samplesOffset + nProcSamples * PARSED_PING_DATA_ITEM_SIZE))
            return false;

        if (CreateProcArray(nProcSamples))
        {
            for (unsigned int i = 0; i < m_procSamples.size(); i++)
            {
                int nSamp = GetUShort(pBuffer);
                m_procSamples[i].m_range = GetSampRange(nSamp, SV);
                int16_t angleCode = GetShort(pBuffer);
                double angle = angleCode / PARSED_ANGLE_SCALE;
                m_procSamples[i].m_angle_s = sin(angle);
                m_procSamples[i].m_angle_c = cos(angle);
                m_procSamples[i].m_amp = GetUShort(pBuffer);
                unsigned char qual = GetUChar(pBuffer);
                m_procSamples[i].m_valid = qual == 0 ? true : false;      // 0 = good quality
            }
        }
        else
            return false;
    }
    else
    {
        unsigned int itemSize = 0;
        switch (m_header.m_sampleType)
        {
        case PingHeader::eFormatPhaseDiff:  itemSize = sizeof(BathySample); break;
        case PingHeader::eFormat4Phase:     itemSize = BathySample4Ph_SIZE; break;
        case PingHeader::eFormatIQ:         itemSize = BathySampleIQ_SIZE; break;
        }
        if (length < (samplesOffset + m_header.m_rxSamples * itemSize))
            return false;

        CreateArray();			// make sure that the array is big enough
        switch (m_header.m_sampleType)
        {
        case PingHeader::eFormatPhaseDiff:
            memcpy(m_samples, pBuffer+samplesOffset, m_header.m_rxSamples * sizeof(BathySample));	// This copies the class in memory, which means we probably get the v_table pointer
            break;
        case PingHeader::eFormat4Phase:
        {
            const char* buffPos = pBuffer + samplesOffset;
            for (unsigned int i = 0; i < m_header.m_rxSamples; i++)
            {
                m_4phSamples[i].ReadBuffer(buffPos);
                buffPos += BathySample4Ph_SIZE;
            }
        }
            break;
        case PingHeader::eFormatIQ:
        {
            const char* buffPos = pBuffer+samplesOffset;
            for (unsigned int i = 0; i < m_header.m_rxSamples; i++)
            {
                m_IQsamples[i].ReadBuffer(buffPos);
                buffPos += BathySampleIQ_SIZE;
            }
        }
            break;
        }
    }

    return status;      // TODO: run check on length etc.
}

// Build a byte array for serialization to disk
void PingData::WriteBuffer(char* pBuffer, unsigned int bufferSize, unsigned int version)
{
    if (bufferSize < Size(version))
        return;

    char* origBuffer = pBuffer;                                // Store buffer check against over-run

    // Make the block header
    int blockHeader[2];
    blockHeader[0] = int(version);
    blockHeader[1] = int(int(Size(version)) - SXF_HEADER_BLOCK_LENGTH);
    memcpy(pBuffer, reinterpret_cast<char*>(blockHeader), sizeof(blockHeader));
    pBuffer += sizeof(blockHeader);

    m_header.SetVersion(version);
    pBuffer = m_header.WriteBuffer(pBuffer, m_procSamples.size());            // Write the header: the new point in the buffer is returned

    switch (version)
    {
    case SONAR_DATA4:
        // All of these sample types could be supported, and we might add new ones soon, to support new processing algorithms
        switch (m_header.m_sampleType)
        {
        case PingHeader::eFormatPhaseDiff:
            if (m_samples != nullptr)
            {
                unsigned int buffsize = m_header.m_rxSamples * sizeof(BathySample);
                memcpy(pBuffer, m_samples, buffsize);
                pBuffer += buffsize;
            }
            break;
        case PingHeader::eFormat4Phase:
            if (m_4phSamples != nullptr)
            {
                unsigned int buffsize = m_header.m_rxSamples * sizeof(BathySample4Ph);
                memcpy(pBuffer, m_4phSamples, buffsize);
                pBuffer += buffsize;
            }
            break;
        case PingHeader::eFormatIQ:
            if (m_IQsamples != nullptr)
            {
                unsigned int buffsize = m_header.m_rxSamples * sizeof(BathySampleIQ);
                memcpy(pBuffer, m_IQsamples, buffsize);
                pBuffer += buffsize;
            }
            break;
        }
        break;
    case PARSED_PING_DATA:
        // Data samples
        for (unsigned int i = 0; i < m_procSamples.size(); i++)
        {
            unsigned int nSamp = GetSampForRange(m_procSamples[i].m_range);
            SetUShort(uint16_t(nSamp), pBuffer);
            int16_t angleCode = int16_t(asin(m_procSamples[i].m_angle_s) * PARSED_ANGLE_SCALE);
            SetShort(angleCode, pBuffer);
            SetUShort(m_procSamples[i].m_amp, pBuffer);
            unsigned char qual = m_procSamples[i].m_valid ? 0 : 1;      // 0 = good quality
            SetUChar(qual, pBuffer);
        }
        break;
    }

    // Check against buffer overrun
    if (uint32_t(pBuffer - origBuffer) > bufferSize)
    {
        qDebug() << "Buffer overrun writing to file";
        return;
    }
}

//-------------------------------------------------
void PingData::EnableTx()
{
    m_header.EnableTx();
}

//-------------------------------------------------
// Empty out the sample data
void PingData::ClearSamples()
{
    switch (m_header.m_sampleType)
    {
    case PingHeader::eFormatPhaseDiff:	if (m_samples != nullptr) std::memset(m_samples, 0, m_arraySize * sizeof(BathySample)); break;
    case PingHeader::eFormat4Phase:		if (m_4phSamples != nullptr) std::memset(m_4phSamples, 0, m_arraySize * sizeof(BathySample4Ph)); break;
    case PingHeader::eFormatIQ:			if (m_IQsamples != nullptr) std::memset(m_IQsamples, 0, m_arraySize * sizeof(BathySampleIQ)); break;
    }
}

//-------------------------------------------------
unsigned int PingData::Size(unsigned int version)
{
    unsigned int bytesToStore = SXF_HEADER_BLOCK_LENGTH;   // Block header
    bytesToStore += m_header.Size(version);

    switch(version)
    {
    case SONAR_DATA4:
        switch (m_header.m_sampleType)
        {
        case PingHeader::eFormatPhaseDiff:	bytesToStore +=  m_arraySize * sizeof(BathySample);     break;
        case PingHeader::eFormat4Phase:		bytesToStore +=  m_arraySize * sizeof(BathySample4Ph);  break;
        case PingHeader::eFormatIQ:			bytesToStore +=  m_arraySize * sizeof(BathySampleIQ);   break;
        }
        break;
    case PARSED_PING_DATA:
        bytesToStore +=  m_arraySize * PARSED_PING_DATA_ITEM_SIZE;
        break;
    }

    return bytesToStore;
}

// Set the data format version number
void PingData::setVersion(unsigned int version)
{
    m_header.SetVersion(version);
}

// Set up the trigger code
void PingData::SetTriggerValues(bool extTrigger, unsigned int extTriggerSource, bool extTriggerInvert,
                                bool triggerOut, unsigned int triggerOutSource, bool triggerOutInvert)
{
    unsigned short value = 0;
    if (extTrigger)
    {
        utilities::setFlagV(value, extTriggerSource, true);
        utilities::setFlagV(value, uint16_t(BSWSonarHardware::trigger_invert), extTriggerInvert ? 0 : 1);
    }
    if (triggerOut)
    {
        utilities::setFlagV(value, uint16_t(triggerOutSource), true);
        utilities::setFlagV(value, uint16_t(BSWSonarHardware::trigger_invert), triggerOutInvert ? 0 : 1);
    }
    m_header.m_triggerFlags = value;
}

// Return text summarising the data
string PingData::getDataSummary()
{
    string summaryString;
    // TODO: complete this
    /*
    double totalAmplitude = 0.;
    int count = 0;


    switch (m_header.m_sampleType)
    {
    case PingHeader::eFormatPhaseDiff:
        if (m_samples != nullptr)
        {
            for (int i = 0; i < m_header.m_rxSamples; i++)
            {

            }
        }
        break;
    case PingHeader::eFormat4Phase:
        if (m_4phSamples != nullptr)
        {
        }
        break;
    case PingHeader::eFormatIQ:
        if (m_IQsamples != nullptr)
        {
        }
        break;
    }
    */

    double sd = 0;
    unsigned int meanAmplitude = GetMeanAmplitude(sd);

    char strBuf[300];
    sprintf(strBuf, "mn amp:%d, sd amp: %.2f", meanAmplitude, sd);
    summaryString = strBuf;
    return summaryString;
}

// Get the amplitude for this sample
unsigned short PingData::GetSampAmplitude(unsigned int iSamp) const
{
    unsigned short amplitude = 0;

    switch (m_header.m_sampleType)
    {
    case PingHeader::eFormatPhaseDiff:
        if (m_samples)
        {
            amplitude = m_samples[iSamp].getAmplitude();
        }
        break;
    case PingHeader::eFormat4Phase:
        if (m_4phSamples)
        {
            amplitude = m_4phSamples[iSamp].m_amp;
        }
        break;
    case PingHeader::eFormatIQ:
        //	TO DO
        break;
    }

    return amplitude;
}

// Get mimimum and maximum amplitude in the array. Ignore samples before minRange.
bool PingData::GetMinMaxAmplitude(double& minAmp, double& maxAmp, double minRange) const
{
    if (m_header.m_sampleType == PingHeader::eFormatIQ)
        return false;       // not supported yet

    minAmp = numeric_limits<double>::max();
    maxAmp = 0.;
    unsigned int startSamp = minRange == 0. ? 0 : GetSampForRange(minRange);

    for (unsigned int i = startSamp; i < GetSampleCount(); i++)
    {
        double amp = double(GetSampAmplitude(i));
        if (amp < minAmp) minAmp = amp;
        if (amp > maxAmp) maxAmp = amp;
     }

    return true;
}

// Get phase data for one sample
bool PingData::GetSampPhases(int iSamp, unsigned char& ab, unsigned char& ac, unsigned char& ad) const
{
    if ((iSamp < 0) || (iSamp >= int(GetSampleCount())))
        return false;
    switch (m_header.m_sampleType)
    {
    case PingHeader::eFormatPhaseDiff:
        ab = m_samples[iSamp].ab;
        ac = m_samples[iSamp].ac;
        ad = m_samples[iSamp].ad;
    break;
    case PingHeader::eFormat4Phase:
    {
        unsigned char phA = uint8_t(m_4phSamples[iSamp].m_phaseA >> 8);
        unsigned char phB = uint8_t(m_4phSamples[iSamp].m_phaseB >> 8);
        unsigned char phC = uint8_t(m_4phSamples[iSamp].m_phaseC >> 8);
        unsigned char phD = uint8_t(m_4phSamples[iSamp].m_phaseD >> 8);
        ab = phA - phB;
        ac = phA - phC;
        ad = phA - phD;
    }
    break;
    case PingHeader::eFormatIQ:
        return false;       // TODO
    }

    return true;
}

// Get the slant range set
double PingData::GetNominalRange() const
{
    return ((m_header.m_rxSamples * double(m_header.m_rxPeriod) + getTimeOffset()) * DEFAULT_SV / 2);
}

// Set the transmit parameters, given a nominal slant range in metres
void PingData::SetNominalRange(double range)
{
    m_header.m_rxSamples = uint32_t(((2 * range / DEFAULT_SV) - getTimeOffset()) / double(m_header.m_rxPeriod));
}

// Get the slant range for a sample
double PingData::GetSampNominalRange(int iSamp) const
{
    return (iSamp * double(m_header.m_rxPeriod) * DEFAULT_SV / 2);
}

// Get the slant range for a sample, using real SV
double PingData::GetSampRange(int iSamp, double SV) const
{
    return ((iSamp * double(m_header.m_rxPeriod) + getTimeOffset()) * SV / 2);
}

// Get the sample number for a range, using nominal SV
unsigned int PingData::GetSampForRange(double range) const
{
    return uint32_t(((2 * range / DEFAULT_SV) - getTimeOffset()) / double(m_header.m_rxPeriod));
}

double PingData::GetNominalFrequency() const
{
    double nominalFrequency = 0;		// zero means "not applicable"
    switch (m_header.m_tdrType)
    {
    case TXD_TYPE_117_V2:
    case TXD_TYPE_117:
        nominalFrequency = TXD_FREQ_117;
        break;
    case TXD_TYPE_234_V2:
    case TXD_TYPE_234:
        nominalFrequency = TXD_FREQ_234;
        break;
    case TXD_TYPE_468_V2:
    case TXD_TYPE_468:
        nominalFrequency = TXD_FREQ_468;
        break;

        // Otherwise use the sonar frequency as a clue, and expect we're using a standard transducer
    case TXD_TYPE_NO_CONN:
//    case TXD_TYPE_DEFAULT:
    default:
        {
            double sonarFrequency = double(m_header.m_operatingFreq);
            if (sonarFrequency != 0.)
            {
                double tolerance = 20e3;	// Looking for a frequency within this number of Hz
                if (abs(sonarFrequency - TXD_FREQ_117) < tolerance)
                    nominalFrequency = TXD_FREQ_117;
                else if (abs(sonarFrequency - TXD_FREQ_234) < tolerance)
                    nominalFrequency = TXD_FREQ_234;
                else if (abs(sonarFrequency - TXD_FREQ_468) < tolerance)
                    nominalFrequency = TXD_FREQ_468;
                // Else we leave it as the input sonar frequency
            }
        }
    }

    return nominalFrequency;
}

// Get the fixed time offset for data samples given the version of the sonar and transmit length
double PingData::getTimeOffset() const
{
    double sonarTypeOffset;
    switch (m_header.m_boardType)
    {
    case BRD_TYPE_117_Q0:
    case BRD_TYPE_117	:
    case BRD_TYPE_ISA	:
    case BRD_TYPE_234	:
    case BRD_TYPE_117_A	:
    case BRD_TYPE_234_A	:
    case BRD_TYPE_468_A	:
    case BRD_TYPE_USB	:
    case BRD_TYPE_MK4	:
        sonarTypeOffset = 0;
        break;
    case BRD_TYPE_V2_A	:
    default:
        sonarTypeOffset = BSWSnr::V2_SAMPLE_TIME_OFFSET;
    }
    double transmitPulseOffset = m_header.m_txCycles * 0.5 / double(m_header.m_operatingFreq);
    double timeOffset = sonarTypeOffset - transmitPulseOffset;
    return timeOffset;
}

void PingData::addProcSample(ProcSample procSample)
{
    m_procSamples.push_back(procSample);
}



// A string summarising the header
string PingData::getSummaryString()
{
    string summaryString;
    char strBuf[300];
    sprintf(strBuf, "#%d, ch:%d, pow:%d, cyc:%d, samp:%d, rng:%.1f ", m_header.m_pingNum, m_header.m_tdrChannel, m_header.m_txPower,
            m_header.m_txCycles, m_header.m_rxSamples, GetNominalRange());
    summaryString = strBuf;
    return summaryString;
}

// **************************************

// We might support any of these sample types in future, or add new ones

BathySample::BathySample()
{
    ab = 0;
    ac = 0;
    ad = 0;
    txno = 0;
    samp0 = 0;
    samp1 = 0;
    anal0 = 0;
    anal1 = 0;
}

BathySample4Ph::BathySample4Ph()
{
    m_sampNo = 0;
    m_phaseA = 0;
    m_phaseB = 0;
    m_phaseC = 0;
    m_phaseD = 0;
    m_amp = 0;
}

// Copy to memory
void BathySample4Ph::WriteBuffer(char* pBuffer)
{
    SetUShort(m_sampNo  , pBuffer);
    SetUShort(m_phaseA  , pBuffer);
    SetUShort(m_phaseB  , pBuffer);
    SetUShort(m_phaseC  , pBuffer);
    SetUShort(m_phaseD  , pBuffer);
    SetUShort(m_amp     , pBuffer);
}

// Read from memory
void BathySample4Ph::ReadBuffer(const char* pBuffer)
{
    m_sampNo    = GetUShort(pBuffer);
    m_phaseA    = GetUShort(pBuffer);
    m_phaseB    = GetUShort(pBuffer);
    m_phaseC    = GetUShort(pBuffer);
    m_phaseD    = GetUShort(pBuffer);
    m_amp       = GetUShort(pBuffer);
}

BathySampleIQ::BathySampleIQ()
{
    m_sampNo = 0;
    m_a_r = 0;
    m_a_i = 0;
    m_b_r = 0;
    m_b_i = 0;
    m_c_r = 0;
    m_c_i = 0;
    m_d_r = 0;
    m_d_i = 0;
};

// Copy to memory
void BathySampleIQ::WriteBuffer(char* pBuffer)
{
    SetUShort(m_sampNo  , pBuffer);
    SetFloat( m_a_r     , pBuffer);
    SetFloat( m_a_i     , pBuffer);
    SetFloat( m_b_r     , pBuffer);
    SetFloat( m_b_i     , pBuffer);
    SetFloat( m_c_r     , pBuffer);
    SetFloat( m_c_i     , pBuffer);
    SetFloat( m_d_r     , pBuffer);
    SetFloat( m_d_i     , pBuffer);
}

// Read from memory
void BathySampleIQ::ReadBuffer(const char* pBuffer)
{
    m_sampNo    = GetUShort(pBuffer);
    m_a_r       = GetFloat(pBuffer);
    m_a_i       = GetFloat(pBuffer);
    m_b_r       = GetFloat(pBuffer);
    m_b_i       = GetFloat(pBuffer);
    m_c_r       = GetFloat(pBuffer);
    m_c_i       = GetFloat(pBuffer);
    m_d_r       = GetFloat(pBuffer);
    m_d_i       = GetFloat(pBuffer);
}

ProcSample::ProcSample()
{
    // polar coordinates at transducer
    m_range     = 0.;			// distance to point - polar in metres
    m_angle_s   = 0.;			// sine of angle to point - polar range -1 to +1
    m_angle_c   = 0.;			// cosine of angle to point - polar range -1 to +1

   m_hrange_ping= 0.;           // Horizontal range in the ping direction.

   m_amp        = 0;            // amplitude value
   m_procAmp    = 0;            // processed amplitude i.e. TVG value

   m_valid      = true;         // filter status
}

// Set values to zero
void ProcSample::setXYZZero()
{
    m_posV.setZero();
    m_posG.setZero();
    m_hrange_ping= 0.;          // Horizontal range in the ping direction.
}


//--------------------------------------------------
PingHeader::PingHeader(void)
{
    Preset();
}

PingHeader::~PingHeader()
{
}

//Write this from GUI thread to avoid inconsistencies
void PingHeader::Set(bool acal, bool on, bool doPing, unsigned int cycles, unsigned int power,
                     float gain, double maxPingRate, int unsigned noSamples, double period, unsigned int adcEnable,
                     bool pps_enable, bool pps_rising, ePingOperation pingOp, bool usePCTime,
                     bool externalTriggerOutPolarity, bool externalTriggerEdge, bool externalTrigger, eSampleType sampleType)
{
    // m_useChan	// This is set later according to the transmit control
    m_calBit			= acal;
    m_rxBit				= on;
    m_pingBit			= doPing;
    m_txCycles			= cycles;
    m_txPower			= power;
    m_analogueGain		= gain;				//unused at the moment, but will keep for calibrated tdr use
    m_rxSamples			= noSamples;
    m_rxPeriod			= float(period);
    m_sidescanAdcEnable = adcEnable;
    m_maxPingRate		= maxPingRate;

    m_PPSProc.EnablePPS(pps_enable ? true : false);
    m_PPSProc.SetPPSEdge(pps_rising ? true : false);
    m_PPSProc.SetUsePCTime(usePCTime ? true : false);

    SetPingMode(pingOp);

    // We don't know port/starboard yet, so can't set that bit

    m_externalTrigger = externalTrigger;
    m_externalTriggerEdge = externalTriggerEdge;
    m_externalTriggerOutPolarity = externalTriggerOutPolarity;

    m_sampleType = sampleType;
}

void PingHeader::SetPingMode(ePingOperation pingOp)
{
    // Mode byte
    m_pingMode = 0;
    switch (pingOp)
    {
    case PO_NONE:						// as said
        m_pingMode = SONAR_SEL_OFF;
        break;

    case PO_SINGLE:						// single transducer
        m_pingMode = SONAR_SEL_SINGLE;
        break;

    case PO_ALTERNATING:				// 2 TEMS used, 1 at a time
        m_pingMode = SONAR_SEL_ALT;
        break;

    case PO_SIMULTANEOUS:				// 2 TEMS used simultaneously
        m_pingMode = SONAR_SEL_SIM;
        break;
    }

    // Transmit on/off
    if (m_pingBit)
    {
        m_pingMode |= SONAR_STATUS_TX_BIT;
    }
}

//--------------------------------------------------
void PingHeader::Preset(void)
{
    m_pingNum			= 0;        // Unique to each measured channel. 2 simultaneous might get 4244, 4245 etc
    m_tdrChannel		= 0;
    m_stavesInTem		= 4;        // All tdrs use 4 as of June 2006
    m_fpgaIdent			= 0;
    m_tdrType			= 0;        // Not identified yet
    m_boardType			= 0;
    m_boardIdent		= 0;
    m_operatingFreq     = float(TXD_FREQ_468);
    m_analogueGain		= 1.0;		// Currently unused; may use for determining absolute values
    m_noClocksIn360		= 256;		// 8 bit in 2.pi
    m_FPGAerror			= 0;
    m_calBit		    = false;
    m_txPower			= 1;
    m_txCycles			= 2;
    m_rxSamples			= 2048;
    m_rxPeriod			= float (65.e-6);	// uS
    m_maxPingRate		= 50;
    //m_sidescanAdcEnable= 0x0f;	// Called Channel in FPGA docs. This selects all 4 ADCs to be summed before reception
    m_sidescanAdcEnable= 0x02;		// Called Channel in FPGA docs. This selects just one channel: a better default than the above. MFG 19/11/07
    m_doTimeUpdate		= 0;		// 1 = command a time update from Tx reg set; 0 = ignore
    m_timeSecPC			= 0;		// seconds
    m_timeMsecPC		= 0;		// mS
    m_timeSecSonar		= 0;		// seconds
    m_timeMsecSonar		= 0;		// mS
    m_firstInScan		= 1;
    m_PPSProc.m_pPPS = &m_PPS;
    m_PPS				= 0;
    m_PPSProc.EnablePPS(false);		// PPS disabled by default
    m_PPSProc.SetUsePCTime(true);	// Use PC time by default
    //m_PPS				|= PingHeader::PPS_DISABLE_BIT;	// PPS disabled by default
    //m_PPS				|= PingHeader::PPS_USE_PC_TIME; // Use PC time by default
    m_pingMode			= 0;
    m_pingBit			= 0;			// Tx - Write only to FPGA; not cleared by FPGA
    m_rxBit				= 0;			// Rx - always set in Usb write operation
    m_resetBit			= 0;			// Write only to FPGA; not cleared by FPGA
    m_version			= SONAR_DATA4;	// Data format identifier
    lastTdrChannel		= -1;			// Not set yet
    m_gapsInData		= false;
    m_ampError			= 0;
    m_externalTrigger	= false;
    m_externalTriggerOutPolarity = true;
    m_externalTriggerEdge = true;

    m_triggerFlags		= 0;			//  0: no triggers
    m_pgagain			= 0;			//	Gain in the Programmable Gain Attenuators
    m_lnagain			= 0;			//	Gain in the Low Noise Amplifiers
    m_basegain			= 0;			//	Gain in 1/100th db at the end of the transmit pulse
    m_lingain			= 0;			//	Amount in 1/100th db/ms the gain rises linearly
    m_sqgain			= 0;			//	Amount in 1/100th db/ms^2 the gain rises as the square of time
    m_swgain			= 0.;			//	Final gain in dB that is applied in software
    m_RxDecimation		= 1;			//	Hardware sends 1 in n samples; 1 = no decimation
    m_rxBandwidth		= 30.;			//	Filter bandwidth in Hz
    m_preampPowerOn		= 0;			//	0 = power off, 1 = on

    m_sampleType		= eFormatPhaseDiff;
}
//-------------------------------------------------
unsigned int PingHeader::Size(unsigned int version)
{
    unsigned int size = 0;
    switch (version)
    {
    case SONAR_DATA4: size = SONAR_DATA4_HEAD_SIZE; break;
    case PARSED_PING_DATA: size = PARSED_PING_DATA_HEADER_SIZE; break;
    }

    return size;
}

//-------------------------------------------------
// Write the ping header to a buffer in memory. Returns the location of the next
// point in the buffer after the header has been written
// We can write various versions
char* PingHeader::WriteBuffer(char* pBuffer, unsigned int nProcSamples, double SV)	// Disk File buffer
{
    // We are likely to add other versions in future, e.g. to support wideband operation
    switch (m_version)
    {

    default:
    case SONAR_DATA4:
    {
        SetUInt(m_pingNum, pBuffer);
        SetUChar(uint8_t(m_tdrChannel), pBuffer);
        SetUChar(uint8_t(m_fpgaIdent), pBuffer);
        SetUChar(uint8_t(m_tdrType), pBuffer);
        SetUChar(uint8_t(m_boardType), pBuffer);
        SetLongLong(m_boardIdent, pBuffer);
        SetFloat(m_operatingFreq, pBuffer);
        SetUChar(uint8_t(m_FPGAerror), pBuffer);
        SetBool(m_calBit, pBuffer);
        SetUChar(uint8_t(m_txPower), pBuffer);
        SetUShort(uint16_t(m_txCycles), pBuffer);
        SetUShort(uint16_t(m_rxSamples), pBuffer);
        SetFloat(m_rxPeriod, pBuffer);  // *
        SetUChar(uint8_t(m_sidescanAdcEnable), pBuffer);
        SetUInt(uint32_t(m_timeSecPC), pBuffer);
        SetUShort(uint16_t(m_timeMsecPC), pBuffer);
        SetUInt(uint32_t(m_timeSecSonar), pBuffer);
        SetUShort(uint16_t(m_timeMsecSonar), pBuffer);
        SetUChar(uint8_t(m_firstInScan), pBuffer);
        SetUChar(uint8_t(m_PPS), pBuffer);
        SetUChar(uint8_t(m_pingMode), pBuffer);
        SetUShort(m_triggerFlags, pBuffer);
        SetUShort(m_pgagain, pBuffer);
        SetUShort(m_lnagain, pBuffer);
        SetUShort(m_basegain, pBuffer);
        SetUShort(m_lingain, pBuffer);
        SetUShort(m_sqgain, pBuffer);
        SetFloat(m_swgain, pBuffer);
        SetUShort(m_RxDecimation, pBuffer);
        SetFloat(m_rxBandwidth, pBuffer);
        SetBool(m_preampPowerOn, pBuffer);
        SetUChar(uint8_t(m_sampleType), pBuffer);
    }
        break;
    case PARSED_PING_DATA:
    {
        SetUInt(m_timeSecSonar, pBuffer);
        SetUInt(m_timeMsecSonar * 1000, pBuffer);
        SetUChar(uint8_t(m_tdrChannel), pBuffer);
        SetUInt(m_pingNum, pBuffer);
        SetFloat(m_operatingFreq, pBuffer);
        SetFloat(float(m_rxPeriod), pBuffer);
        SetUShort(uint16_t(nProcSamples), pBuffer);
        SetFloat(float(SV), pBuffer);
        SetUShort(uint16_t(m_txCycles), pBuffer);
        unsigned int qualOption = QUALITY_FILTER_FLAGS;
        unsigned char dataOptions = qualOption & 0x7;
        SetUChar(dataOptions, pBuffer);
        unsigned char pingState = uint8_t(SONAR_SEL_SIM);     	//Simultaneous pinging (TODO: set the correct mode)
        SetUChar(pingState, pBuffer);
        SetUShort(uint16_t(m_rxSamples), pBuffer);              // Count before filtering

        // ... then PARSED_RESERVED_BYTES reserved bytes
        memset(pBuffer, 0, PARSED_RESERVED_BYTES);
        pBuffer += PARSED_RESERVED_BYTES;
    }
        break;
    }

    return pBuffer;		// Pass back the point where we've got to so far
}

void PingHeader::ReadBuffer(const char * pBuffer, double& SV, unsigned int& nProcSamples, eQualityOptions& qualOption)
{
    SV = 0.;
    nProcSamples = 0;
    qualOption = QUALITY_FILTER_FLAGS;

    switch (m_version)
    {
    case SONAR_DATA:
    case SONAR_DATA2:
    case SONAR_DATA2_TEST:
    case SONAR_DATA3:
    case SONAR_DATA4A:
        cout << "ERROR: old SONAR_DATA block types not handled in this code" << endl;
        // This will break, because we aren't moving the buffer forwards. But we shouldn't see any SONAR_DATA blocks anyway.
        break;

    case SONAR_DATA4:
    {
        m_pingNum			= GetUInt(pBuffer);
        m_tdrChannel		= uint32_t(GetUChar(pBuffer));
        m_fpgaIdent			= uint32_t(GetUChar(pBuffer));
        m_tdrType			= uint32_t(GetUChar(pBuffer));
        m_boardType			= uint32_t(GetUChar(pBuffer));
        m_boardIdent		= GetLongLong(pBuffer);
        m_operatingFreq		= GetFloat(pBuffer);
        m_FPGAerror			= uint32_t(GetUChar(pBuffer));
        m_calBit			= GetBool(pBuffer);
        m_txPower			= uint32_t(GetUChar(pBuffer));
        m_txCycles			= uint32_t(GetShort(pBuffer));
        m_rxSamples			= uint32_t(GetUShort(pBuffer));
        m_rxPeriod			= GetFloat(pBuffer);
        m_sidescanAdcEnable	= uint32_t(GetUChar(pBuffer));
        m_timeSecPC			= uint32_t(GetInt(pBuffer));
        m_timeMsecPC		= uint32_t(GetUShort(pBuffer));
        m_timeSecSonar		= uint32_t(GetUInt(pBuffer));
        m_timeMsecSonar		= uint32_t(GetUShort(pBuffer));
        m_firstInScan		= uint32_t(GetUChar(pBuffer));
        m_PPS				= GetUChar(pBuffer);
        m_pingMode			= uint32_t(GetUChar(pBuffer));

        m_triggerFlags		= GetUShort(pBuffer);
        m_pgagain			= GetUShort(pBuffer);
        m_lnagain			= GetUShort(pBuffer);
        m_basegain			= GetUShort(pBuffer);
        m_lingain			= GetUShort(pBuffer);
        m_sqgain			= GetUShort(pBuffer);
        m_swgain			= GetFloat(pBuffer);
        m_RxDecimation		= GetUShort(pBuffer);
        m_rxBandwidth		= GetFloat(pBuffer);
        m_preampPowerOn		= GetBool(pBuffer);
        m_sampleType		= PingHeader::eSampleType(*pBuffer++);
    } // 73 bytes
        break;

    case PARSED_PING_DATA:
    {
        m_timeSecSonar = GetUInt(pBuffer);
        m_timeMsecSonar = GetUInt(pBuffer) / 1000;
        m_tdrChannel = GetUChar(pBuffer);
        m_pingNum = GetUInt(pBuffer);
        m_operatingFreq = GetFloat(pBuffer);
        m_rxPeriod = GetFloat(pBuffer);
        nProcSamples = GetUShort(pBuffer);
        SV = double(GetFloat(pBuffer));
        m_txCycles = GetUShort(pBuffer);
        unsigned char dataOptions = GetUChar(pBuffer);
        qualOption = eQualityOptions(dataOptions & 0x7);
        m_pingMode = GetUChar(pBuffer);
        m_rxSamples = GetUShort(pBuffer);              // Count before filtering
        // ... then PARSED_RESERVED_BYTES reserved bytes
        pBuffer += PARSED_RESERVED_BYTES;
    }
    break;

    default:
        // Could handle an error condition here ...
        return;
    }
}

//-----------------------------------------------
void PingHeader::SetVersion(unsigned int aVer)
{
    m_version = aVer;
}


//-----------------------------------------------
// This ought to be moved into the swath project
void PingHeader::TextLog(std::string& textlog)
{
    char text[200];

    textlog += "Sonar settings:\r\n";

    (m_pingBit) ? textlog += "Ping On, " : textlog += "Ping Off, ";

    (m_calBit) ? textlog += "Amplfier test on " : textlog += "Amplifier test off ";
    textlog += "\r\n";

    sprintf(text, "Transmit length: %d, ", m_txCycles);
    textlog += text;
    sprintf(text, "Transmit power: %d, ", m_txPower);
    textlog += text;
    textlog += "\r\n";

    sprintf(text, "Analogue channel/s: %d, ", m_sidescanAdcEnable);
    textlog += text;
    sprintf(text, "Ping frequency %.3f, ", double(m_operatingFreq));
    textlog += text;
    sprintf(text, "Receive length: %d, " , m_rxSamples);
    textlog += text;
    sprintf(text, "Receive period: %.2fus", double(m_rxPeriod) * 1e6);
    textlog += text;
    textlog += "\r\n";
}

//-----------------------------------------------
void PingHeader::EnableRx(void)
{
    m_rxBit = true;
}
//-----------------------------------------------
void PingHeader::DisableRx(void)
{
    m_rxBit = false;
}
//-----------------------------------------------
bool PingHeader::RxIsEnabled(void)
{
    return m_rxBit;
}

//-----------------------------------------------
void PingHeader::EnableTx(void)
{
    m_pingBit = true;
    m_pingMode |= SONAR_STATUS_TX_BIT;
}
//-----------------------------------------------
void PingHeader::DisableTx(void)
{
    m_pingBit = false;
    m_pingMode &= uint32_t(~SONAR_STATUS_TX_BIT);
}
//-----------------------------------------------
bool PingHeader::TxIsEnabled(void)
{
    return m_pingBit;
}
