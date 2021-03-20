#ifndef BATHY_SONAR_ROS_SWATH_PROC_H
#define BATHY_SONAR_ROS_SWATH_PROC_H

#include <cstdlib>
#include <string>

namespace soslab {

    class SwathProcess {
    public:
        SwathProcess() = default;

        void start();

        void stop();

        void setConfigPath(std::string p) { m_config_path = p; }

    private:
        std::string m_config_path;
        pid_t m_swathRT_pid;
    };

}

#endif //BATHY_SONAR_ROS_SWATH_PROC_H
