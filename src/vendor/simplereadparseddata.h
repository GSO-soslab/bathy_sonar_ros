// A minimum reader of parsed data files for demonstration purposes

#ifndef SIMPLEREADPARSEDDATA_H
#define SIMPLEREADPARSEDDATA_H

// Qt
#include <QByteArray>

// std
#include <vector>

#include "sample.h"

#include "sonar_data_header.h"

class SimpleReadParsedData
{
private:

    bool m_isParsed;
    std::vector<sample> m_samples;
    bool m_readingBlock;                            // Currently reading a data block
    unsigned int m_readingBlockType;                // The type of block being read
    QByteArray m_UDPreadBuffer;                     // Buffer of bytes read from UDP
    unsigned int m_UDPreadExpect;                   // Number of bytes to read in the current block

    sonar_data_header m_sonarHeader;

public:
    SimpleReadParsedData();
    
    bool readUDPData(QByteArray* ba);
    bool ReadBuffer(const char * pBuffer, unsigned int length);
    double GetSampRange(int iSamp, double SV) const;                // Get the slant range for a sample, using real SV
    double getTimeOffset() const;                                   // Get the fixed time offset for data samples given the version of the sonar and transmit length

    bool getIsParsed() { return m_isParsed; }

    sonar_data_header getSonarHeader() { return m_sonarHeader; }

    std::vector<sample> getSamples();


};

#endif // SIMPLEREADPARSEDDATA_H
