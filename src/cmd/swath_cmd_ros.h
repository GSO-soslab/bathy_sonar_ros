#ifndef BATHY_SONAR_ROS_SWATH_CMD_ROS_H
#define BATHY_SONAR_ROS_SWATH_CMD_ROS_H

// ROS
#include "ros/ros.h"
#include "std_srvs/Trigger.h"

// SOSlab
#include "swath_cmd.h"
#include "swath_com.h"
#include "swath_proc.h"

namespace soslab {
    
    class SwathCmdRos {

    public:
        SwathCmdRos();

        void initialize();

        void destroy();

    private:
   
        bool startSonar(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response& res);
      
        bool stopSonar(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res);
        
        bool enableTx(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res);

        bool disableTx(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res);
        
        bool enableUdp(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res);

        bool disableUdp(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res);
        
    private:
        std::shared_ptr<SwathCmd> m_swathCmd;

        std::shared_ptr<SwathCom> m_swathCom;

        SwathProcess m_swathProcess;
       
        ros::NodeHandle m_nh;
        
        ros::NodeHandle m_pnh;
    
        ros::ServiceServer m_start_sonar_service;
        
        ros::ServiceServer m_stop_sonar_service;
    
        ros::ServiceServer m_enable_tx_service;
        
        ros::ServiceServer m_disable_tx_service;
       
        ros::ServiceServer m_enable_udp_service;
        
        ros::ServiceServer m_disable_udp_service;
    };
    
}

#endif //BATHY_SONAR_ROS_SWATH_CMD_ROS_H
