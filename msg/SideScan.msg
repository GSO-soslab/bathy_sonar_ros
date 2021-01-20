Header header 

uint32 test_sec
uint32 test_msec

int8 channel
uint16 ping_num
float32 operate_freq #Sonar frequency of transducer, in Hz, interpreted from FPGA registers.
float32 rx_period  #?
float64 sv #?
uint16 tx_cycles # Length of the transmit pulse, in sonar cycles

uint16 sample_size
float64[] ranges
float64[] angle
float64[] angle_s
float64[] angle_c
uint16[] amplitude
bool[] valid