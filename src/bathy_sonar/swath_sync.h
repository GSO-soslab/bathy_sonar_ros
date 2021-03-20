#ifndef BATHY_SONAR_ROS_SWATH_SYNC_H
#define BATHY_SONAR_ROS_SWATH_SYNC_H

#include <QObject>
#include <QTimer>
#include <QThread>

#include "swath_cmd.h"

namespace soslab {
    class SwathSync : public QObject
    {
        Q_OBJECT
    public:
        explicit SwathSync(std::shared_ptr<SwathCmd> cmd);
        SwathSync(const SwathSync &_o) = default;
        ~SwathSync() override = default;

    public slots:
        void sendZDA();

    signals:
        void resultReady(const QString &result);

    private:
        QTimer *timer;
        std::shared_ptr<SwathCmd> cmd;
    };

}
#endif //BATHY_SONAR_ROS_SWATH_SYNC_H
