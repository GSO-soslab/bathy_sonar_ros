#include "swath_com.h"

#include "../vendor/simplereadparseddata.h"

#include <qt5/QtNetwork/QUdpSocket>

#include <iostream>

#include "ros/ros.h"
#include "swath_cmd_ros.h"

using namespace soslab;

SwathCom::SwathCom() : QObject(nullptr)
{
    m_udpsocket = std::make_shared<QUdpSocket>(this);
    if (!m_udpsocket->bind(QHostAddress(QHostAddress::AnyIPv4), EXT_RX_PORT, QUdpSocket::ShareAddress)) {
        ROS_WARN_STREAM("could not bind UDP socket to port " << EXT_RX_PORT);
    }
    QObject::connect(m_udpsocket.get(), &QUdpSocket::readyRead,this, &SwathCom::sonarReadyRead);
    
    m_parser = std::make_shared<SimpleReadParsedData>();
}

SwathCom::SwathCom(const SwathCom &o) : QObject(nullptr) {
    m_udpsocket = o.m_udpsocket;
    m_ha = o.m_ha;
}

void SwathCom::sonarSendRequest(const QByteArray &ba_, const QHostAddress &ha_)
{
    if (m_udpsocket->writeDatagram(ba_, ha_, EXT_TX_PORT) != ba_.size()) {
        ROS_WARN_STREAM("could not write all bytes");
    }
}

void SwathCom::sonarReadyRead()
{
    QByteArray ba;
    QHostAddress addr;
    quint16 port;
    qint64 dgsize;

    while (m_udpsocket->hasPendingDatagrams()) {
        dgsize = m_udpsocket->pendingDatagramSize();
        ba.resize(dgsize);
        m_udpsocket->readDatagram(ba.data(), dgsize, &addr, &port);
        // emit sonarAnswerReceived(ba, addr);

        bool result = m_parser->readUDPData(&ba);
        if(m_parser->getIsParsed()) {
            m_ros->publishSonarData(m_parser->getSamples(), m_parser->getSonarHeader());
            m_ros->publishPointCloud(m_parser->getSamples());
        }
        // printf("data: %s\n", ba.data());
        /* parse range and angle to ros msg and publish */
    }
}

void SwathCom::m_parse() {

}
