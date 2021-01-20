#pragma once
/// Hardware interface structures used to communicate between Bathyswath-2 embedded software
/// on TEM and external computers

namespace BSWSonarHardware {

/// Constants, describing the hardware
    static const unsigned int nTdcrs = 3;                   // Number of transducers
    static const unsigned int nStaves = 4;                  // Number of staves per channel
    static const unsigned int nChans = nTdcrs * nStaves;    // Total number of channels in the TEM
    static const double V2_sampleTimeOffset = 213.5;         // V2 systems, time offset to received samples, microseconds

// The ADC sample rate is 40MHz from a crystal, then decimate by 8, decimate by 8 and decimate by 8 in the three filter stages.
// So, 40e6 / (8*8*8) = 78125 Hz
    static const unsigned int V2baseSampleRate = 78125;


/// Data packets passed between software and TEM

#pragma pack(1)		// pack on byte boundaries

// This header is prepended to all messages to and from the Bathyswath hardware
    struct bathyswath_header {
        static const int identLength = 10;
        unsigned char ident[identLength];   // "Bathyswath"
        unsigned short spare;
        unsigned int serial_number;         // Serial number of the TEM
        unsigned int protocol_version;
        unsigned int packet_type;           // See enum pkt_types
        unsigned int packet_length;         // Total length of the packet, including this header
    };

// Information about transducers. Not used in the BSW-2 software, and not implemented in BSW-3 hardware.
    struct transducer_data {
        unsigned int transducer_sn;
        unsigned short transducer_frequency;
        unsigned short transducer_hw_revision;
    };

// Message sent by UDP broadcast (i.e. to all subnets) by the Bathyswath hardware, nominally once a minute
    struct bathyswath_broadcast
    {
        unsigned short hw_major_version;         // Version of the TEM hardware (FPGA)
        unsigned short hw_minor_version;
        unsigned short sw_major_version;         // Version of the TEM software (on NIOS core or SoC processor)
        unsigned short sw_minor_version;
        struct transducer_data transducers[nTdcrs];   // Information about the transducers
        unsigned int timeval_increment_rate;     // Rate (in Hz) at which the timeval is incremented, nominally 1 Hz (so that timeval is in seconds)
        static const unsigned int numTemperatures = 8;
        char temperatures[numTemperatures];      // Readings of the temperature sensors on the FPGA and PA boards
        unsigned short pa_voltage;               // Voltage of the HT supply on the PA board (in volts?)
        unsigned int system_status;              // Status of the TEM; see Sonar::sonarstatus
        unsigned long long last_pps_time_s;      // Time at which the latest PPS message was received, seconds
        unsigned int last_pps_time_us;           // Time at which the latest PPS message was received, microseconds
        unsigned long long current_time_s;       // Current time, seconds
        unsigned int current_time_us;            // Current time, microseconds
        unsigned int pps_error_count;            // Number of PPS errors recorded
        unsigned int pps_good_count;             // Number of good PPS pulses received
    };

// Specifies what data type is being sent by the bathyswath_config packet
    enum configuration_commands {
        config_ip_address = 0,          // Set the IP address (in fixed IP mode)
        config_tx_attenuation,          // Set transmit power (PA HT voltage);
        config_bite_frequency,          // Set frequency used to inject test signals to TEM and transducer
        config_bite_mode,               // Set BITE mode
        config_time_val,                // Set time
        config_flags,                   // Set various flags
        config_preamp_power,            // Turn transducer preamp power on and off
        config_dhcp,                    // Enable and disable DHCP Ethernet IP addressing
        config_serial_number,           // Set the serial number into EEPROM in the TEM
        config_pps,                     // Configure PPS reception
    };

/// Mode control flags
    enum eControlFlags {
        txdr_one,                       // = 1, Ping on transducer 1
        txdr_two,                       // = 2, Ping on transducer 2
        txdr_three,                     // = 4, Ping on transducer 3
        trigger_gpi1,                   // = 8, Trigger on GPI2 rising edge
        trigger_gpi2,                   // = 16, Trigger on GPI2 rising edge
        trigger_rs485_1,                // = 32, Trigger on RS485 1 rising edge
        trigger_rs485_2,                // = 64, Trigger on RS485 2 rising edge
        trigger_invert,                 // = 128, Trigger on falling instead of rising edge
        trigger_out_gpi1,               // = 256, Output Triggers
        trigger_out_gpi2,               // = 512, Trigger on GPI2 rising edge
        trigger_out_rs485_1,            // = 1024, Trigger on RS485 1 rising edge
        trigger_out_rs485_2,            // = 2048, Trigger on RS485 2 rising edge
        trigger_out_invert,             // = 4096, Trigger on falling instead of rising edge
    };

// Set up PPS reception in the TEM
    struct pps_config{
        unsigned int source;            // Source of PPS data; 0: RS485 0, 1: RS485 1, 2: GPI 0, 3: GPI 1
        unsigned int invert;            // Rising or falling edge
        unsigned int sync_enable;       // Enable use of PPS (0 = off, 1 = on)
        unsigned int pps_window;        // PPS valid window length in us (valid PPS pulses must be received within 1s +- this window after the previous one);
    };

// Packet to configure the TEM; consists of a "what" enum value, followed by one of the list of config data types.
    struct bathyswath_config {
        enum configuration_commands what;
        union {
            unsigned int set_ip_address;                // Not currently used. The embedded code in the TEM uses set_ip[] instead.
            unsigned int set_transmit_attenuation;      // transmit power; 15 = no power, 0 = max power
            unsigned int set_bite_frequency;            // Frequency to use for BITE signal injection, kHz
            unsigned int set_bite_mode;                 // BITE signal injection mode. See Sonar::eBite_controls; 0 = off, 1 = tdcr, 2 = front end amplifiers
            unsigned int set_time_val[2];               // Time to set in the TEM clock. High and low parts of a time in microseconds, lower 44bits are seconds, upper 44bits are microseconds, using little-endian.
            unsigned int set_flags;                     // Doesn't currently seem to be used anywhere
            unsigned char set_ip[8];                    // IP address to use if in fixed IP mode (not in DHCP mode)
            unsigned int set_preamp_power;              // Turn 12V power to transducer preamps on and off; 0 = off
            unsigned int dhcp;                          // Enable DHCP IP addressing (0 = off)
            unsigned int serial_number;                 // Set the serial number of the FPGA board
            struct pps_config pps_config;
        } data;
    };

// Ask the TEM to send out a sonar ping, using the parameters listed.
    struct bathyswath_ping_req {
        unsigned int flags;                 // See eControlFlags
        short pgagain;                      // Gain of the PGA preamp (after TVG). In dBs x 100; options are 2400 (24dB) & 3000 (30dB)
        short lnagain;                      // Gain of the LNA preamp (front end). In dBs x 100; options are 1200 (12dB), 1800 (18dB), 2400 (24dB)
        // basegain, lingain & sqgain work together to set the TVG. The maximum of the total is 40dB.
        // The operation of the TVG in firmware is currently a mystery, and it will be re-written.
        // In particular, the time period of the linear and square gains is not specified.
        short basegain;                     // Base gain of the TVG; gain that doesn't change with time. In dB x 100, so max. 4000.
        short lingain;                      // Size of linear gain of the TVG; increases linearly with time. In dB x 100, so max. 4000.
        short sqgain;                       // Size of square gain of the TVG; increases with the square of time. In dB x 100, so max. 4000.
        short aqu_time;                     // Length of the ping (range) in milliseconds. (1000 * 2 * range / nominalSV)
        unsigned int frequency[nTdcrs];     // Sonar frequency of the channels in kHz
        unsigned int tx_cycles[nTdcrs];     // Transmit pulse length of the channels in cycles
        unsigned char bandwidth;            // Filter bandwidth setting. Options are 0 (10kHz), 1 (30kHz), 2 (100kHz)
        unsigned char decimation;           // Reduce the size of the output data by sending 1 in n samples
        unsigned short pingid;              // Ping number; the controlling software increments this before sending the ping request, and it becomes the ping number of the received ping
    };

// Complex data value for returned data
    struct iq_pair {
        int i;
        int q;
    };

    static const int PKT_DATA_LEN = 160;    // Number of data samples sent in each bathyswath_ping_response packet

// Ping data returned from the TEM
// Each ping is sent from the TEM as a stream of these packets
    struct bathyswath_ping_response
    {
        // Lower and upper parts of the time stamp. Lower 44bits are seconds, upper 44bits are microseconds, using little-endian
        unsigned int timeval_lo;
        unsigned int timeval_hi;
        unsigned short channel;             // Sonar channel, 0 to 11 (3 tdcrs x 4 staves)
        unsigned short seqnum;              // Sequence number within the channel; the first item in data[] is at position seqnum x PKT_DATA_LEN.
        unsigned int pingid;                // Ping number, as set in bathyswath_ping_req
        unsigned int data_length;           // Total length of the recorded data
        iq_pair data[PKT_DATA_LEN];         // The raw sonar data, as complex numbers
    };

// A data packet to or from the TEM; consists of a header and one of the listed data structures.
    struct bathyswath_packet {
        bathyswath_header header;
        union {
            bathyswath_broadcast broadcast;
            bathyswath_config config;
            bathyswath_ping_req ping_req;
            bathyswath_ping_response ping_response;
        } data;
    };

// Packet type; put in the packet_type member of bathyswath_header
    enum pkt_types {
        pkt_broadcast_announce = 0,
        pkt__sonar_configuration,
        pkt_ping_req,
        pkt_ping_response
    };

#pragma pack()				// Restore normal packing

/// End of data packet definitions

}; // namespace BSWSonarHardware
