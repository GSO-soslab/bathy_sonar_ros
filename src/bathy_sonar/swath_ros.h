#ifndef BATHY_SONAR_ROS_SWATH_ROS_H
#define BATHY_SONAR_ROS_SWATH_ROS_H

// ROS
#include <src/vendor/sample.h>
#include <src/vendor/sonar_data_header.h>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "definitions.h"

namespace soslab {
    class SwathCmd;
    class SwathProcess;
    class SwathSync;

    class SwathRos : public std::enable_shared_from_this<SwathRos> {

    private:
        std::shared_ptr<SwathCmd> m_swathCmd;

        std::shared_ptr<SwathSync> m_swathSync;

        std::shared_ptr<SwathProcess> m_swathProcess;

        ros::NodeHandle m_nh;

        ros::NodeHandle m_pnh;

        std::string frame_id;

        std::string channel_1_frame_id;

        std::string channel_2_frame_id;

        ros::ServiceServer m_start_sonar_service;

        ros::ServiceServer m_stop_sonar_service;

        ros::ServiceServer m_enable_tx_service;

        ros::ServiceServer m_disable_tx_service;

        ros::ServiceServer m_enable_udp_service;

        ros::ServiceServer m_disable_udp_service;

        ros::Publisher m_sonarPublisher;

        ros::Publisher m_pointCloudPublisher;

    public:
        SwathRos();

        void initialize();

        void destroy();

        void publish();

        void publishPointCloud(std::vector<sample> _samples);

        void publishSonarData(std::vector<sample> _samples, sonar_data_header _header);

    private:
   
        bool startSonar(std_srvs::Trigger::Request& _req,
                        std_srvs::Trigger::Response& _res);
      
        bool stopSonar(std_srvs::Trigger::Request& _req,
                       std_srvs::Trigger::Response& _res);
        
        bool enableTx(std_srvs::Trigger::Request& _req,
                      std_srvs::Trigger::Response& _res);

        bool disableTx(std_srvs::Trigger::Request& _req,
                       std_srvs::Trigger::Response& _res);
        
        bool enableUdp(std_srvs::Trigger::Request& _req,
                      std_srvs::Trigger::Response& _res);

        bool disableUdp(std_srvs::Trigger::Request& _req,
                       std_srvs::Trigger::Response& _res);



    };
    
}

#endif //BATHY_SONAR_ROS_SWATH_ROS_H
