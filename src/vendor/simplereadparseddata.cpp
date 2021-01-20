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
            default:
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
            if ((m_isParsed = ReadBuffer(m_UDPreadBuffer.constData(), m_UDPreadExpect)))
            {
                // Finished with this ping packet; get ready for the next one
                m_readingBlock = false;
                m_UDPreadBuffer.clear();
            }
            else {
                status = false;
            }
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
    m_sonarHeader.timeSecSonar = GetUInt(pBuffer);
    m_sonarHeader.timeMsecSonar = GetUInt(pBuffer) / 1000;
    m_sonarHeader.tdrChannel = GetUChar(pBuffer);
    m_sonarHeader.pingNum = GetUInt(pBuffer);
    m_sonarHeader.operatingFreq = GetFloat(pBuffer);
    m_sonarHeader.rxPeriod = GetFloat(pBuffer);
    m_sonarHeader.nProcSamples = GetUShort(pBuffer);
    m_sonarHeader.sv = GetFloat(pBuffer);
    m_sonarHeader.txCycles = GetUShort(pBuffer);
    m_sonarHeader.dataOptions = GetUChar(pBuffer);
    eQualityOptions qualOption = eQualityOptions(m_sonarHeader.dataOptions & 0x7);
    m_sonarHeader.pingMode = GetUChar(pBuffer);
    m_sonarHeader.rxSamples = GetUShort(pBuffer);              // Count before filtering
    // ... then PARSED_RESERVED_BYTES reserved bytes
    pBuffer += PARSED_RESERVED_BYTES;
    
    // Check the total size of the buffer is OK
    if (length < (PARSED_PING_DATA_HEADER_SIZE + m_sonarHeader.nProcSamples * PARSED_PING_DATA_ITEM_SIZE))
        return false;
    
    m_samples.clear();
    m_samples.resize(m_sonarHeader.nProcSamples);
    
    for (int i = 0; i < m_sonarHeader.nProcSamples; i++)
    {
        int nSamp = GetUShort(pBuffer);
        m_samples[i].m_range = GetSampRange(nSamp, m_sonarHeader.sv);
        int16_t angleCode = GetShort(pBuffer);
        double angle = angleCode / PARSED_ANGLE_SCALE;
        m_samples[i].m_angle_s = sin(angle);
        m_samples[i].m_angle_c = cos(angle);
        m_samples[i].m_amp = GetUShort(pBuffer);
        unsigned char qual = GetUChar(pBuffer);
        m_samples[i].m_valid = qual == 0;      // 0 = good quality
    }

    return true;
}

// Get the slant range for a sample, using real SV
double SimpleReadParsedData::GetSampRange(int iSamp, double SV) const
{
    return ((iSamp * double(m_sonarHeader.rxPeriod) + getTimeOffset()) * SV / 2);
}

// Get the fixed time offset for data samples given the version of the sonar and transmit length
double SimpleReadParsedData::getTimeOffset() const
{
    double sonarTypeOffset = BSWSnr::V2_SAMPLE_TIME_OFFSET;
    double transmitPulseOffset = m_sonarHeader.txCycles * 0.5 / double(m_sonarHeader.operatingFreq);
    double timeOffset = sonarTypeOffset - transmitPulseOffset;
    return timeOffset;
}
