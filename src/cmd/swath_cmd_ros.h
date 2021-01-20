#ifndef BATHY_SONAR_ROS_SWATH_CMD_ROS_H
#define BATHY_SONAR_ROS_SWATH_CMD_ROS_H

// ROS
#include <src/vendor/sample.h>
#include <src/vendor/sonar_data_header.h>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"


namespace soslab {
    class SwathCmd;
    class SwathCom;
    class SwathProcess;

    class SwathCmdRos : public std::enable_shared_from_this<SwathCmdRos> {

    private:
        std::shared_ptr<SwathCmd> m_swathCmd;

        std::shared_ptr<SwathCom> m_swathCom;

        std::shared_ptr<SwathProcess> m_swathProcess;

        ros::NodeHandle m_nh;

        ros::NodeHandle m_pnh;

        ros::ServiceServer m_start_sonar_service;

        ros::ServiceServer m_stop_sonar_service;

        ros::ServiceServer m_enable_tx_service;

        ros::ServiceServer m_disable_tx_service;

        ros::ServiceServer m_enable_udp_service;

        ros::ServiceServer m_disable_udp_service;

        ros::Publisher m_sonarPublisher;

        ros::Publisher m_pointCloudPublisher;

    public:
        SwathCmdRos();

        void initialize();

        void destroy();

        void publish();

        void publishPointCloud(std::vector<sample> samples);

        void publishSonarData(std::vector<sample> samples, sonar_data_header header);

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



    };
    
}

#endif //BATHY_SONAR_ROS_SWATH_CMD_ROS_H
