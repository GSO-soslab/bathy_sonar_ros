#ifndef BATHY_SONAR_ROS_SWATH_CMD_H
#define BATHY_SONAR_ROS_SWATH_CMD_H

// std
#include <string>
#include <memory>
#include <QObject>

// Forward declarations
class QUdpSocket;
class QHostAddress;
class QByteArray;

#define TEST_UDP_RX_PORT (52765)
#define TEST_UDP_TX_PORT (52763)

namespace soslab {
    
    class SwathCmd : public QObject {
        Q_OBJECT
    public:
        explicit SwathCmd();
      
        SwathCmd(const SwathCmd &o);

        ~SwathCmd() override = default;

        void initialize();
        
        void startSonar();
    
        void stopSonar();
   
        void resetSonar();
    
        void killSonar();
        
        void testTrigger();
   
        void setUdpState(bool state);
        
        void setRange(int range);
        
        void setTxState(bool state);

        bool testCommunication();

    protected slots:
        void m_readyRead();
        
    private slots:

        void m_setupRemote();
        
        void m_sendMessage(const QByteArray &ba);
        
        void m_setTargetIp(QString ha);
    
        void m_sendNMEAMessage(const char* talker, const char* type, const char* data);
        
        void m_sendNMEAMessage(const char* talker, const char* type, const QString data);
    
        static int m_calcChecksum(std::string message);
        
        static int m_calcChecksum(QString message);
        
        static void m_checksumChars(int calcChecksum, char* checksum_str);
        
    private:
        // Current commanded state of the remote system
        bool m_txOn;
        bool m_sendUDP;
        bool m_writeToFile;
        int m_range;
        
        //! @brief: IP address of the sonar
        QString m_IPAddress;
        
        //! @todo: It's not clear what this does
        QString m_folderName;
    
        std::shared_ptr<QUdpSocket> m_udpSocket;
    
        std::shared_ptr<QHostAddress> m_ha;

    };
    
}


#endif //BATHY_SONAR_ROS_SWATH_CMD_H
