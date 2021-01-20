#ifndef BATHY_SONAR_ROS_SWATH_COM_H
#define BATHY_SONAR_ROS_SWATH_COM_H

// Qt
#include <QObject>

// Std
#include <memory>

// Vendor
#include "../vendor/simplereadparseddata.h"



#define EXT_RX_PORT (52764)
#define EXT_TX_PORT (52763)

class QUdpSocket;
class QHostAddress;

namespace soslab {

    class SwathCmdRos;

    class SwathCom : public QObject {
    Q_OBJECT
    public:
        explicit SwathCom();

        ~SwathCom() override = default;

        SwathCom(const SwathCom &o);

        void setRos(std::shared_ptr<SwathCmdRos> ros) { m_ros = ros; }

    signals:
        void sonarAnswerReceived(QByteArray &ba_, const QHostAddress &addr_);
    
    public slots:
        void sonarSendRequest(const QByteArray &ba_, const QHostAddress &ha_);
    
    protected slots:
        void sonarReadyRead();
        

    
    private:
        void m_parse();

        std::shared_ptr<SwathCmdRos> m_ros;

        std::shared_ptr<QUdpSocket> m_udpsocket;
        std::shared_ptr<QHostAddress> m_ha;
        std::shared_ptr<SimpleReadParsedData> m_parser;
    };
}

#endif //BATHY_SONAR_ROS_SWATH_COM_H
