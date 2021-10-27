#ifndef BATHY_SONAR_ROS_SWATH_CMD_H
#define BATHY_SONAR_ROS_SWATH_CMD_H

#include "definitions.h"

// std
#include <string>
#include <memory>
#include <QObject>

// Vendor
#include "../vendor/simplereadparseddata.h"

// Forward declarations
class QUdpSocket;
class QHostAddress;
class QByteArray;

namespace soslab {

    class SwathRos;

    class SwathCmd : public QObject {
        Q_OBJECT
    public:
        explicit SwathCmd();
      
        SwathCmd(const SwathCmd &_o);

        ~SwathCmd() override = default;

        void initialize();

        void sendMessage(const QByteArray _ba);

        bool startSonar();
    
        bool stopSonar();
   
        bool resetSonar();
    
        bool killSonar();
        
        bool testTrigger();
   
        bool setUdpState(bool _state);
        
        bool setRange(int _range);
        
        bool setTxState(bool _state);

        void setRos(std::shared_ptr<SwathRos> _ros) { m_ros = _ros; }

        bool testCommunication();

    protected slots:
        void m_readyRead();
        
    private slots:

        bool m_setupRemote();
        
        bool m_sendMessage(const QByteArray &_ba);
        
        void m_setTargetIp(QString _ha);
    
        bool m_sendNMEAMessage(const char* _talker, const char* _type, const char* _data);
        
        bool m_sendNMEAMessage(const char* _talker, const char* _type, const QString _data);
    
        static int m_calcChecksum(std::string _msg);
        
        static int m_calcChecksum(QString _msg);
        
        static void m_checksumChars(int _calcChecksum, char* _checksumStr);
        
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

        std::shared_ptr<SimpleReadParsedData> m_parser;

        std::shared_ptr<SwathRos> m_ros;
    };
    
}


#endif //BATHY_SONAR_ROS_SWATH_CMD_H
