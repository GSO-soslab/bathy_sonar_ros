#include <thread>
#include "swath_cmd_ros.h"
#include "swath_proc.h"
#include "swath_cmd.h"
#include "swath_com.h"
#include "bathy_sonar_ros/SideScan.h"
#include "ros/package.h"
#include "sensor_msgs/PointCloud2.h"

using namespace soslab;

SwathCmdRos::SwathCmdRos() :
    m_pnh("~")
{
    m_swathCmd = std::make_shared<SwathCmd>();

    m_swathCom = std::make_shared<SwathCom>();

    m_swathProcess = std::make_shared<SwathProcess>();

    m_start_sonar_service = m_pnh.advertiseService("start_sonar", &SwathCmdRos::startSonar, this);
    m_stop_sonar_service = m_pnh.advertiseService("stop_sonar", &SwathCmdRos::stopSonar, this);
    m_enable_tx_service = m_pnh.advertiseService("enable_tx", &SwathCmdRos::enableTx, this);
    m_disable_tx_service = m_pnh.advertiseService("disable_tx", &SwathCmdRos::disableTx, this);
    m_enable_udp_service = m_pnh.advertiseService("enable_udp", &SwathCmdRos::enableUdp, this);
    m_disable_udp_service = m_pnh.advertiseService("disable_udp", &SwathCmdRos::disableUdp, this);

    std::string path = ros::package::getPath("bathy_sonar_ros");
    std::string configPath;
    m_pnh.param<std::string>("swath_rt_config_file", configPath, path + "/config/swathRT/RTsettings.ini");
    m_swathProcess->setConfigPath(configPath);


    m_sonarPublisher = m_pnh.advertise<bathy_sonar_ros::SideScan>("sidescan",1000);

    m_pointCloudPublisher = m_pnh.advertise<sensor_msgs::PointCloud2>("pointcloud",1000);

}

void SwathCmdRos::initialize() {
    std::shared_ptr<SwathCmdRos> that = shared_from_this();
    m_swathCom->setRos(that);
    m_swathProcess->start();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    m_swathCmd->startSonar();
    m_swathCmd->setTxState(true);
    m_swathCmd->setUdpState(true);
}

void SwathCmdRos::destroy() {
    m_swathProcess->stop();
}


bool SwathCmdRos::startSonar(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res) {
    m_swathCmd->startSonar();
    // todo: fill the response
    return true;
}

bool SwathCmdRos::stopSonar(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res) {
    m_swathCmd->stopSonar();
    // todo: fill the response
    return true;
}

bool SwathCmdRos::enableTx(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res) {
    m_swathCmd->setTxState(true);
    // todo: fill the response
    return true;
}

bool SwathCmdRos::disableTx(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res) {
    m_swathCmd->setTxState(false);
    // todo: fill the response
    return true;
}

bool SwathCmdRos::enableUdp(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res) {
    m_swathCmd->setUdpState(true);
    // todo: fill the response
    return true;
}

bool SwathCmdRos::disableUdp(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res) {
    m_swathCmd->setUdpState(false);
    // todo: fill the response
    return true;
}

void SwathCmdRos::publish() {

}

void SwathCmdRos::publishPointCloud(std::vector<sample> samples) {
    const uint32_t POINT_STEP = 16;
    sensor_msgs::PointCloud2 msg;
/*
    // if time synchronized, use sonar time, otherwise just use ros time now
    if(config.time_sync){
        msg.header.stamp = scan->m_sonarHeader.stamp;

        duration = ros::Time::now() - scan->m_sonarHeader.stamp;
        deltaTime.data = duration.sec + duration.nsec/1e9;
    }
    else{
        msg.header.stamp = ros::Time::now();

        deltaTime.data = 0.0;
    }

    pubDeltaTime.publish(deltaTime);
*/
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "sidescan";

    msg.fields.resize(4);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[3].count = 1;
    msg.data.resize(std::max((size_t)1, samples.size()) * POINT_STEP, 0x00);
    msg.point_step = POINT_STEP;
    msg.row_step = msg.data.size();
    msg.height = 1;
    msg.width = msg.row_step / POINT_STEP;
    uint8_t *ptr = msg.data.data();

    for(const auto& sample : samples) {
        if(sample.m_valid) {
            *(reinterpret_cast<float*>(ptr + 0)) = sample.m_range * sample.m_angle_c;
            *(reinterpret_cast<float*>(ptr + 4)) = 0;
            *(reinterpret_cast<float*>(ptr + 8)) = sample.m_range * sample.m_angle_s;
            *(reinterpret_cast<float*>(ptr + 12)) = sample.m_amp * 1e-4;
            ptr += POINT_STEP;
        }
    }

    m_pointCloudPublisher.publish(msg);
/*
    if(scan->channel == 1){
        msg.header.frame_id = "port_transducer";
        pubPort.publish(msg); // channel 1
    }
    else{
        msg.header.frame_id = "strd_transducer";
        pubStrd.publish(msg); // channel 2
`
    }
*/
}

void SwathCmdRos::publishSonarData(std::vector<sample> samples, sonar_data_header header) {
    bathy_sonar_ros::SideScan msg;
    msg.test_sec = header.timeSecSonar;
    msg.test_msec = header.timeMsecSonar;
    msg.channel = header.tdrChannel;
    msg.ping_num = header.pingNum;
    msg.operate_freq = header.operatingFreq;
    msg.rx_period = header.rxPeriod;
    msg.sv = header.sv;
    msg.tx_cycles = header.txCycles;
    msg.sample_size = header.nProcSamples;


    msg.ranges.resize(samples.size());
    msg.angle_c.resize(samples.size());
    msg.angle_s.resize(samples.size());
    msg.amplitude.resize(samples.size());
    msg.valid.resize(samples.size());

    for(int i = 0 ; i < samples.size() ; i++) {
        msg.ranges[i] = samples[i].m_range;
        msg.angle_c[i] = samples[i].m_angle_c;
        msg.angle_s[i] = samples[i].m_angle_s;
        msg.amplitude[i] = samples[i].m_amp;
        msg.valid[i] = samples[i].m_valid;
    }

    m_sonarPublisher.publish(msg);

}