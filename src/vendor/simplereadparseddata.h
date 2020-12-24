// A minimum reader of parsed data files for demonstration purposes

#ifndef SIMPLEREADPARSEDDATA_H
#define SIMPLEREADPARSEDDATA_H

// Qt
#include <QByteArray>

// std
#include <vector>

#include "sample.h"

class SimpleReadParsedData
{
public:
    SimpleReadParsedData();
    
    bool readUDPData(QByteArray* ba);
    bool ReadBuffer(const char * pBuffer, unsigned int length);
    double GetSampRange(int iSamp, double SV) const;                // Get the slant range for a sample, using real SV
    double getTimeOffset() const;                                   // Get the fixed time offset for data samples given the version of the sonar and transmit length

    std::vector<sample> getSamples();
    
private:
    
    std::vector<sample> m_samples;
    bool m_readingBlock;                            // Currently reading a data block
    unsigned int m_readingBlockType;                // The type of block being read
    QByteArray m_UDPreadBuffer;                     // Buffer of bytes read from UDP
    unsigned int m_UDPreadExpect;                   // Number of bytes to read in the current block
    
    float m_rxPeriod;
    unsigned int m_txCycles;        // Length of the transmit pulse, in sonar cycles
    float	m_operatingFreq;		// Sonar frequency of transducer, in Hz, interpreted from FPGA registers.
    
};

#endif // SIMPLEREADPARSEDDATA_H
