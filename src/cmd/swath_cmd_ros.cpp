#include "swath_cmd_ros.h"

using namespace soslab;

SwathCmdRos::SwathCmdRos() :
    m_pnh("~")
{
    m_swathCmd = std::make_shared<SwathCmd>();
    m_swathCom = std::make_shared<SwathCom>();

    m_start_sonar_service = m_pnh.advertiseService("start_sonar", &SwathCmdRos::startSonar, this);
    m_stop_sonar_service = m_pnh.advertiseService("stop_sonar", &SwathCmdRos::stopSonar, this);
    m_enable_tx_service = m_pnh.advertiseService("enable_tx", &SwathCmdRos::enableTx, this);
    m_disable_tx_service = m_pnh.advertiseService("disable_tx", &SwathCmdRos::disableTx, this);
    m_enable_udp_service = m_pnh.advertiseService("enable_udp", &SwathCmdRos::enableUdp, this);
    m_disable_udp_service = m_pnh.advertiseService("disable_udp", &SwathCmdRos::disableUdp, this);
}

void SwathCmdRos::initialize() {
    m_swathProcess.start();
}

void SwathCmdRos::destroy() {
    m_swathProcess.stop();
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
