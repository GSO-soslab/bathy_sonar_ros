#include "swath_cmd.h"

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
        if (!m_udpSocket->bind(QHostAddress(QHostAddress::Any), TEST_UDP_RX_PORT)) {
            ROS_WARN_STREAM("could not bind UDP socket to port " << TEST_UDP_RX_PORT << ", try: " << i);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            break;
        }
    }
    connect(m_udpSocket.get(), &QUdpSocket::readyRead, this, &SwathCmd::m_readyRead);
}

SwathCmd::SwathCmd(const SwathCmd &o) : QObject(nullptr) {
    m_udpSocket = o.m_udpSocket;
    m_ha = o.m_ha;
    m_txOn = o.m_txOn;
    m_sendUDP = o.m_sendUDP;
    m_writeToFile = o.m_writeToFile;
    m_range = o.m_range;
    m_IPAddress = o.m_IPAddress;
}

void SwathCmd::initialize() {

}

void SwathCmd::m_sendMessage(const QByteArray &byteArray) {
    if(m_udpSocket->writeDatagram(byteArray, *m_ha, TEST_UDP_TX_PORT) != byteArray.size()) {
        qDebug() << "could not write all bytes";
    }
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
        qDebug() << "ready read, bytes read: " << ba.size() << "from" << addr.toString() << ":" << port;
    }
}

void SwathCmd::m_setupRemote() {
    m_sendNMEAMessage("SW", "PCT", QString("SNR,TX,") + (m_txOn ? "ON" : "OFF"));
    m_sendNMEAMessage("SW", "PCT", QString("UDP,SEND,") + (m_sendUDP ? "ON" : "OFF"));
    m_sendNMEAMessage("SW", "PCT", QString("FILE,WRITE,") + (m_writeToFile ? "ON" : "OFF"));
    m_sendNMEAMessage("SW", "PCT", QString("SNR,RNG,") + QString::number(m_range));
    m_sendNMEAMessage("SW", "PCT", QString("FILE,FLDR,") + m_folderName);
}

int SwathCmd::m_calcChecksum(QString message) {
    return  m_calcChecksum(message.toStdString());
}

int SwathCmd::m_calcChecksum(std::string message) {
    int calcChecksum = 0;
    
    if (message.size() != 0)
    {
        // Must start with '$'
        if (message[0] == '$')
        {
            std::string::size_type csPos = message.find_last_of("*");
            if (csPos != std::string::npos)
            {
                // All characters between the second (after the '$') and up to
                // but not including the '*'
                unsigned char csChar;							// character for checksumming
                for (unsigned short i = 1; i < csPos; i++)
                {
                    csChar = message.at(i);
                    calcChecksum ^= csChar;
                }
            }
        }
    }
    return calcChecksum;
}

void SwathCmd::m_sendNMEAMessage(const char *talker, const char *type, const char *data) {
    m_sendNMEAMessage(talker, type, QString(data));
}

void SwathCmd::m_sendNMEAMessage(const char *talker, const char *type, const QString data) {
    QString message;
    message += "$";
    message += talker;
    message += type;
    message += ",";
    message += data;
    message += "*";
    int checksum = m_calcChecksum(message);
    char checksum_str[3];
    m_checksumChars(checksum, checksum_str);
    message += checksum_str;
    message += "\r\n";
    m_sendMessage(message.toLatin1());
}

void SwathCmd::m_checksumChars(int calcChecksum, char *checksum_str) {
    sprintf(checksum_str, "%02X", calcChecksum);
}

void SwathCmd::m_setTargetIp(QString ha) {
    m_IPAddress = ha;
    m_ha->setAddress(ha);
}

void SwathCmd::testTrigger() {
    m_sendNMEAMessage("SW", "PCT", "TEST_TRIGGER");
}

void SwathCmd::setUdpState(bool state) {
    m_sendUDP = state;
    m_sendNMEAMessage("SW", "PCT", QString("UDP,SEND,") + (state ? "ON" : "OFF"));
}

void SwathCmd::setTxState(bool state) {
    m_txOn= state;
    m_sendNMEAMessage("SW", "PCT", QString("SNR,TX,") + (state ? "ON" : "OFF"));
}

void SwathCmd::startSonar() {
    m_setupRemote();
    m_sendNMEAMessage("P", "MISS", "LINE_START");
}

void SwathCmd::stopSonar() {
    m_sendNMEAMessage("P", "MISS", "LINE_END");
}

void SwathCmd::resetSonar() {
    m_sendNMEAMessage("SW", "PCT", "SNR,RESET");
}

void SwathCmd::killSonar() {
    m_sendNMEAMessage("SW", "PCT", "KILL");
}

void SwathCmd::setRange(int range) {
    m_range = range;
    m_sendNMEAMessage("SW", "PCT", QString("SNR,RNG,") + QString::number(m_range));
}

bool SwathCmd::testCommunication() {
    return false;
}

