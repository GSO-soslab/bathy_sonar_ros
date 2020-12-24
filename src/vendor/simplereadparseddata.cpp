#include "simplereadparseddata.h"
#include "BathyswathFileDefs.h"
#include "BathyswathSonarDefs.h"
#include "utilities.h"
#include "sample.h"
#include <vector>

using namespace utilities;
using namespace std;

SimpleReadParsedData::SimpleReadParsedData()
{
    m_readingBlock = false;
    m_UDPreadExpect = 0;
}

// Handle data from the UDP socket
bool SimpleReadParsedData::readUDPData(QByteArray* ba)
{
    bool status = false;
    unsigned int blockHeader[2];
    
    // swathRT breaks data from each ping into a number of smaller UDP packets, as UDP
    // packets shouldn't be too big for efficient data transfer.
    // So, we need to stitch them back together into one data block for each ping.
    // Like all Bathyswath data packets, the ping packet starts with an identifiable header,
    // so we look for that.
    
    if (!m_readingBlock)
    {
        // Is this packet the start of a data block?
        memcpy(reinterpret_cast<char*>(blockHeader), ba->constData(), sizeof(blockHeader));    // Copy the first part of the block into the blockheader
        unsigned int blockType = blockHeader[0];
        unsigned int bufferSize = blockHeader[1];
        switch(blockType)
        {
            case PARSED_PING_DATA:
                if ((bufferSize > 0) && (bufferSize < SXF_MAXBLOCKSIZE))
                {
                    m_readingBlock = true;
                    m_readingBlockType = blockType;
                    m_UDPreadExpect = bufferSize;
                    m_UDPreadBuffer.clear();
                    m_UDPreadBuffer.append(ba->mid(sizeof(blockHeader)));       // Add the data in the UDP packet to the ping buffer, skipping over the data packet buffer
                    status = true;
                }
                break;
        }
    }
    else
    {
        // Currently reading in a block
        m_UDPreadBuffer.append(*ba);                                        // Add the data in the UDP packet to the ping buffer
        status = true;
        if (uint32_t(m_UDPreadBuffer.size()) >= m_UDPreadExpect)
        {
            // We have read in enough data to make a ping
            if (ReadBuffer(m_UDPreadBuffer.constData(), m_UDPreadExpect))
            {
                // Finished with this ping packet; get ready for the next one
                m_readingBlock = false;
                m_UDPreadBuffer.clear();
            }
            else
                status = false;
        }
    }
    
    return status;
}

std::vector<sample> SimpleReadParsedData::getSamples() {
    return m_samples;
}

bool SimpleReadParsedData::ReadBuffer(const char * pBuffer, unsigned int length)
{
    // Read the header information
    unsigned int timeSecSonar = GetUInt(pBuffer);
    unsigned int timeMsecSonar = GetUInt(pBuffer) / 1000;
    unsigned int tdrChannel = GetUChar(pBuffer);
    unsigned int pingNum = GetUInt(pBuffer);
    m_operatingFreq = GetFloat(pBuffer);
    m_rxPeriod = GetFloat(pBuffer);
    unsigned int nProcSamples = GetUShort(pBuffer);
    double SV = double(GetFloat(pBuffer));
    m_txCycles = GetUShort(pBuffer);
    unsigned char dataOptions = GetUChar(pBuffer);
    eQualityOptions qualOption = eQualityOptions(dataOptions & 0x7);
    unsigned int pingMode = GetUChar(pBuffer);
    unsigned int rxSamples = GetUShort(pBuffer);              // Count before filtering
    // ... then PARSED_RESERVED_BYTES reserved bytes
    pBuffer += PARSED_RESERVED_BYTES;
    
    // Check the total size of the buffer is OK
    if (length < (PARSED_PING_DATA_HEADER_SIZE + nProcSamples * PARSED_PING_DATA_ITEM_SIZE))
        return false;
    
    
    vector<sample> samples;
    samples.resize(nProcSamples);
    
    for (int i = 0; i < nProcSamples; i++)
    {
        int nSamp = GetUShort(pBuffer);
        samples[i].m_range = GetSampRange(nSamp, SV);
        int16_t angleCode = GetShort(pBuffer);
        double angle = angleCode / PARSED_ANGLE_SCALE;
        samples[i].m_angle_s = sin(angle);
        samples[i].m_angle_c = cos(angle);
        samples[i].m_amp = GetUShort(pBuffer);
        unsigned char qual = GetUChar(pBuffer);
        samples[i].m_valid = qual == 0;      // 0 = good quality
    }
    
    
    return true;
}

// Get the slant range for a sample, using real SV
double SimpleReadParsedData::GetSampRange(int iSamp, double SV) const
{
    return ((iSamp * double(m_rxPeriod) + getTimeOffset()) * SV / 2);
}

// Get the fixed time offset for data samples given the version of the sonar and transmit length
double SimpleReadParsedData::getTimeOffset() const
{
    double sonarTypeOffset = BSWSnr::V2_SAMPLE_TIME_OFFSET;
    double transmitPulseOffset = m_txCycles * 0.5 / double(m_operatingFreq);
    double timeOffset = sonarTypeOffset - transmitPulseOffset;
    return timeOffset;
}
