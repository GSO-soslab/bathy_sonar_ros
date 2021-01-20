
#include "ros/ros.h"
#include "src/cmd/swath_cmd_ros.h"

#include <qcoreapplication.h>
#include <thread>
#include <csignal>

using namespace soslab;

std::shared_ptr<SwathCmdRos> controller;

void sighandler(int sig) {
    if(ros::ok()) {
        controller->destroy();
        ros::shutdown();
    }

    QCoreApplication::quit();
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "bathy_ros_node", ros::init_options::NoSigintHandler);
    QCoreApplication app(argc, argv);

    signal(SIGINT, sighandler);

    controller = std::make_shared<SwathCmdRos>();
    controller->initialize();

    auto rosSpin = std::thread([](){
        ros::spin();
    });

    return QCoreApplication::exec();;
}