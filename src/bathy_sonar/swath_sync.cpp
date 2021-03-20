#include "swath_sync.h"
#include "ros/ros.h"
#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o


using namespace soslab;

SwathSync::SwathSync(std::shared_ptr<SwathCmd> c) : cmd(c), QObject(nullptr){
    timer = new QTimer(nullptr);
    connect(timer, &QTimer::timeout, this, &SwathSync::sendZDA);
    timer->start(1000);
}

void SwathSync::sendZDA()
{
    QString result;

    // trig timer to start at the beginging of each 1 s + 50 ms
    // send time message around 50 ms after PPS arrived
    timer->stop();
    ros::Time current = ros::Time::now();
    float delay = (1000 - current.nsec * 1e-6) + 50;
    timer->start(delay);

    // get time
    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    const char *c = iso_time_str.c_str();

    // get specific time
    char year[10], month[10],day[10], hour[10], min[10], sec[10], milli[10];
    sscanf(c, "%4s-%2s-%2sT%2s:%2s:%2s.%2s", year, month, day, hour, min, sec, milli);

    // fill with ZDA time msg
    char nema_zda [100];
    char end [5];
    int i=0, size=0, checksum=0;
    // real sent: 2019/04/04,15:41:13 -> $GPZDA,154113.00,04,04,2019,0,0
    size = sprintf(nema_zda,"$GPZDA,%s%s%s.%s,%s,%s,%s,%d,%d", hour,min,sec,"00",day,month,year,0,0);

    // fill end: checksum, <CR>, <LF>
    for (i = 1; i < size; i ++)
        checksum ^= nema_zda[i];
    sprintf(end, "*%hhx%c%c",checksum,13,10);

    // fill the entire nema zda structure
    strcat(nema_zda,end);

    // send to UDP
    QByteArray Data;
    Data.append(nema_zda);

    cmd->sendMessage(Data);

    // TEST:
    // std::cout<<" Test zda entire: "<< nema_zda<<std::endl;

    emit resultReady(result);
}