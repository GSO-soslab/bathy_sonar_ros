#include <thread>
#include "swath_ros.h"
#include "swath_proc.h"
#include "swath_cmd.h"
#include "swath_sync.h"
#include "bathy_sonar_ros/SideScan.h"
#include "ros/package.h"
#include "sensor_msgs/PointCloud2.h"

using namespace soslab;

SwathRos::SwathRos() :
    m_pnh("~")
{
    m_swathCmd = std::make_shared<SwathCmd>();

    m_swathSync = std::make_shared<SwathSync>(m_swathCmd);

    m_swathProcess = std::make_shared<SwathProcess>();

    m_start_sonar_service = m_pnh.advertiseService("start_sonar", &SwathRos::startSonar, this);
    m_stop_sonar_service = m_pnh.advertiseService("stop_sonar", &SwathRos::stopSonar, this);
    m_enable_tx_service = m_pnh.advertiseService("enable_tx", &SwathRos::enableTx, this);
    m_disable_tx_service = m_pnh.advertiseService("disable_tx", &SwathRos::disableTx, this);
    m_enable_udp_service = m_pnh.advertiseService("enable_udp", &SwathRos::enableUdp, this);
    m_disable_udp_service = m_pnh.advertiseService("disable_udp", &SwathRos::disableUdp, this);

    std::string path = ros::package::getPath("bathy_sonar_ros");
    std::string configPath;

    m_pnh.param<std::string>("frame_id", frame_id, "sidescan");
    m_pnh.param<std::string>("channel_1_frame_id", channel_1_frame_id, "channel_1");
    m_pnh.param<std::string>("channel_2_frame_id", channel_2_frame_id, "channel_2");

    m_pnh.param<std::string>("swath_rt_config_file", configPath, path + "/config/swathRT/RTsettings.ini");
    m_swathProcess->setConfigPath(configPath);


    m_sonarPublisher = m_pnh.advertise<bathy_sonar_ros::SideScan>("sidescan",1000);

    m_pointCloudPublisher = m_pnh.advertise<sensor_msgs::PointCloud2>("pointcloud",1000);

}

void SwathRos::initialize() {

    bool run_swath_rt;
    m_pnh.param<bool>("run_swath_rt", run_swath_rt, false);
    std::shared_ptr<SwathRos> that = shared_from_this();
    if (run_swath_rt) {
        m_swathProcess->start();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    m_swathCmd->setRos(that);
    m_swathCmd->startSonar();
    m_swathCmd->setTxState(true);
    m_swathCmd->setUdpState(true);
}

void SwathRos::destroy() {
    m_swathProcess->stop();
}


bool SwathRos::startSonar(std_srvs::Trigger::Request& _req,
                          std_srvs::Trigger::Response& _res) {
    m_swathCmd->startSonar();
    // todo: fill the response
    return true;
}

bool SwathRos::stopSonar(std_srvs::Trigger::Request &_req,
                         std_srvs::Trigger::Response &_res) {
    m_swathCmd->stopSonar();
    // todo: fill the response
    return true;
}

bool SwathRos::enableTx(std_srvs::Trigger::Request &_req,
                        std_srvs::Trigger::Response &_res) {
    m_swathCmd->setTxState(true);
    // todo: fill the response
    return true;
}

bool SwathRos::disableTx(std_srvs::Trigger::Request &_req,
                         std_srvs::Trigger::Response &_res) {
    m_swathCmd->setTxState(false);
    // todo: fill the response
    return true;
}

bool SwathRos::enableUdp(std_srvs::Trigger::Request &_req,
                         std_srvs::Trigger::Response &_res) {
    m_swathCmd->setUdpState(true);
    // todo: fill the response
    return true;
}

bool SwathRos::disableUdp(std_srvs::Trigger::Request &_req,
                          std_srvs::Trigger::Response &_res) {
    m_swathCmd->setUdpState(false);
    // todo: fill the response
    return true;
}

void SwathRos::publish() {

}

void SwathRos::publishPointCloud(std::vector<sample> _samples) {
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
    msg.header.frame_id = frame_id;

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
    msg.data.resize(std::max((size_t)1, _samples.size()) * POINT_STEP, 0x00);
    msg.point_step = POINT_STEP;
    msg.row_step = msg.data.size();
    msg.height = 1;
    msg.width = msg.row_step / POINT_STEP;
    uint8_t *ptr = msg.data.data();

    for(const auto& sample : _samples) {
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
        msg.header.frame_id = channel_1_frame_id;
        pubPort.publish(msg); // channel 1
    }
    else{
        msg.header.frame_id = channel_2_frame_id;
        pubStrd.publish(msg); // channel 2
    }
*/
}

void SwathRos::publishSonarData(std::vector<sample> _samples, sonar_data_header _header) {
    bathy_sonar_ros::SideScan msg;
    msg.test_sec = _header.timeSecSonar;
    msg.test_msec = _header.timeMsecSonar;
    msg.channel = _header.tdrChannel;
    msg.ping_num = _header.pingNum;
    msg.operate_freq = _header.operatingFreq;
    msg.rx_period = _header.rxPeriod;
    msg.sv = _header.sv;
    msg.tx_cycles = _header.txCycles;
    msg.sample_size = _header.nProcSamples;


    msg.ranges.resize(_samples.size());
    msg.angle_c.resize(_samples.size());
    msg.angle_s.resize(_samples.size());
    msg.amplitude.resize(_samples.size());
    msg.valid.resize(_samples.size());

    for(int i = 0 ; i < _samples.size() ; i++) {
        msg.ranges[i] = _samples[i].m_range;
        msg.angle_c[i] = _samples[i].m_angle_c;
        msg.angle_s[i] = _samples[i].m_angle_s;
        msg.amplitude[i] = _samples[i].m_amp;
        msg.valid[i] = _samples[i].m_valid;
    }

    m_sonarPublisher.publish(msg);

}