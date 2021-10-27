#include "swath_cmd.h"
#include "swath_ros.h"

#include <qt5/QtNetwork/QUdpSocket>
#include <qt5/QtNetwork/QHostAddress>
#include <qt5/QtCore/QByteArray>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QDateTime>
#include <QObject>

#include <iostream>
#include <thread>

#include "ros/ros.h"

using namespace soslab;

SwathCmd::SwathCmd() : QObject(nullptr)
{
    m_udpSocket = std::make_shared<QUdpSocket>();
    m_ha = std::make_shared<QHostAddress>();
    m_txOn = false;
    m_sendUDP = false;
    m_writeToFile = false;
    m_range = 50;
    m_IPAddress = "127.0.0.1";
    
    m_setTargetIp(m_IPAddress);
    m_udpSocket = std::make_shared<QUdpSocket>();
    for(int i = 0 ; i < 10 ; i++) {
        if (!m_udpSocket->bind(QHostAddress(QHostAddress::Any), EXT_RX_PORT)) {
            ROS_WARN_STREAM("could not bind UDP socket to port " << TEST_UDP_RX_PORT << ", try: " << i);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            break;
        }
    }
    connect(m_udpSocket.get(), &QUdpSocket::readyRead, this, &SwathCmd::m_readyRead);

    m_parser = std::make_shared<SimpleReadParsedData>();
}

SwathCmd::SwathCmd(const SwathCmd &_o) : QObject(nullptr) {
    m_udpSocket = _o.m_udpSocket;
    m_ha = _o.m_ha;
    m_txOn = _o.m_txOn;
    m_sendUDP = _o.m_sendUDP;
    m_writeToFile = _o.m_writeToFile;
    m_range = _o.m_range;
    m_IPAddress = _o.m_IPAddress;
}

void SwathCmd::initialize() {

}

bool SwathCmd::m_sendMessage(const QByteArray &_ba) {
    if(m_udpSocket->writeDatagram(_ba, *m_ha, EXT_TX_PORT) != _ba.size()) {
        qDebug() << "could not write all bytes";
        return false;
    } else {
        return true;
    }
}

void SwathCmd::sendMessage(const QByteArray _ba) {
    m_sendMessage(_ba);
}

void SwathCmd::m_readyRead() {
    QByteArray ba;
    QHostAddress addr;
    uint16_t port;
    int64_t dgsize;
    while(m_udpSocket->hasPendingDatagrams()) {
        dgsize = m_udpSocket->pendingDatagramSize();
        ba.resize(dgsize);
        m_udpSocket->readDatagram(ba.data(), dgsize, &addr, &port);
        m_parser->readUDPData(&ba);
        if(m_parser->getIsParsed()) {
            m_ros->publishSonarData(m_parser->getSamples(), m_parser->getSonarHeader());
            m_ros->publishPointCloud(m_parser->getSamples(), m_parser->getSonarHeader());
        }
    }
}

bool SwathCmd::m_setupRemote() {
    bool result = true;
    result &= m_sendNMEAMessage("SW", "PCT", QString("SNR,TX,") + (m_txOn ? "ON" : "OFF"));
    result &= m_sendNMEAMessage("SW", "PCT", QString("UDP,SEND,") + (m_sendUDP ? "ON" : "OFF"));
    result &= m_sendNMEAMessage("SW", "PCT", QString("FILE,WRITE,") + (m_writeToFile ? "ON" : "OFF"));
    result &= m_sendNMEAMessage("SW", "PCT", QString("SNR,RNG,") + QString::number(m_range));
    result &= m_sendNMEAMessage("SW", "PCT", QString("FILE,FLDR,") + m_folderName);
    return result;
}

int SwathCmd::m_calcChecksum(QString _msg) {
    return m_calcChecksum(_msg.toStdString());
}

int SwathCmd::m_calcChecksum(std::string _msg) {
    int calcChecksum = 0;
    
    if (_msg.size() != 0)
    {
        // Must start with '$'
        if (_msg[0] == '$')
        {
            std::string::size_type csPos = _msg.find_last_of("*");
            if (csPos != std::string::npos)
            {
                // All characters between the second (after the '$') and up to
                // but not including the '*'
                unsigned char csChar;							// character for checksumming
                for (unsigned short i = 1; i < csPos; i++)
                {
                    csChar = _msg.at(i);
                    calcChecksum ^= csChar;
                }
            }
        }
    }
    return calcChecksum;
}

bool SwathCmd::m_sendNMEAMessage(const char *_talker, const char *_type, const char *_data) {
    return m_sendNMEAMessage(_talker, _type, QString(_data));
}

bool SwathCmd::m_sendNMEAMessage(const char *_talker, const char *_type, const QString _data) {
    QString message;
    message += "$";
    message += _talker;
    message += _type;
    message += ",";
    message += _data;
    message += "*";
    int checksum = m_calcChecksum(message);
    char checksum_str[3];
    m_checksumChars(checksum, checksum_str);
    message += checksum_str;
    message += "\r\n";
    return m_sendMessage(message.toLatin1());
}

void SwathCmd::m_checksumChars(int _calcChecksum, char *_checksumStr) {
    sprintf(_checksumStr, "%02X", _calcChecksum);
}

void SwathCmd::m_setTargetIp(QString _ha) {
    m_IPAddress = _ha;
    m_ha->setAddress(_ha);
}

bool SwathCmd::testTrigger() {
    return m_sendNMEAMessage("SW", "PCT", "TEST_TRIGGER");
}

bool SwathCmd::setUdpState(bool _state) {
    m_sendUDP = _state;
    return m_sendNMEAMessage("SW", "PCT", QString("UDP,SEND,") + (_state ? "ON" : "OFF"));
}

bool SwathCmd::setTxState(bool _state) {
    m_txOn= _state;
    return m_sendNMEAMessage("SW", "PCT", QString("SNR,TX,") + (_state ? "ON" : "OFF"));
}

bool SwathCmd::startSonar() {
    if(!m_setupRemote()) {
        return false;
    }
    return m_sendNMEAMessage("P", "MISS", "LINE_START");
}

bool SwathCmd::stopSonar() {
    return m_sendNMEAMessage("P", "MISS", "LINE_END");
}

bool SwathCmd::resetSonar() {
    return m_sendNMEAMessage("SW", "PCT", "SNR,RESET");
}

bool SwathCmd::killSonar() {
    return m_sendNMEAMessage("SW", "PCT", "KILL");
}

bool SwathCmd::setRange(int _range) {
    m_range = _range;
    return m_sendNMEAMessage("SW", "PCT", QString("SNR,RNG,") + QString::number(m_range));
}

bool SwathCmd::testCommunication() {
    return false;
}

