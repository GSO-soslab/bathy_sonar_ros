#ifndef BATHY_SONAR_ROS_SWATH_SYNC_H
#define BATHY_SONAR_ROS_SWATH_SYNC_H

#include <QObject>
#include <QTimer>
#include <QThread>

namespace soslab {
    class SwathSync : public QObject
    {
        Q_OBJECT
    public:
        explicit SwathSync(QObject *parent = 0);

        void sendUDP(const QByteArray &ba);

    public slots:
        void sendZDA();

    signals:
        void resultReady(const QStrin &result);

    private:
        QTimer *timer;
        QUdpSocket *socket;
    };

}
#endif //BATHY_SONAR_ROS_SWATH_SYNC_H
