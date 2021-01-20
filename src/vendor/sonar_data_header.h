#ifndef BATHY_SONAR_ROS_SONAR_DATA_HEADER_H
#define BATHY_SONAR_ROS_SONAR_DATA_HEADER_H

struct sonar_data_header {
    uint32_t timeSecSonar;
    uint32_t timeMsecSonar;
    unsigned char tdrChannel;
    uint32_t pingNum;
    float	operatingFreq;		// Sonar frequency of transducer, in Hz, interpreted from FPGA registers.
    float rxPeriod;
    uint32_t nProcSamples;
    float sv;
    unsigned int txCycles;        // Length of the transmit pulse, in sonar cycles
    unsigned char dataOptions;
    uint32_t pingMode;
    uint32_t rxSamples;
};

#endif //BATHY_SONAR_ROS_SONAR_DATA_HEADER_H
