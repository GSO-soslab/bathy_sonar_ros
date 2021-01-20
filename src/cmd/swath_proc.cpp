#include <poll.h>
#include <csignal>
#include "swath_proc.h"
#include "unistd.h"
#include "ros/console.h"

using namespace soslab;

void SwathProcess::start() {

    m_swathRT_pid = fork();
    if(m_swathRT_pid == 0) {
        char *args[] = {
                const_cast<char *>("swathRT"),
                const_cast<char *>(m_config_path.empty() ? "" : "-s"),
                const_cast<char *>(m_config_path.empty() ? "" : m_config_path.c_str()),
        };

        std::string command;
        for(auto i : args) {
            command += " " + std::string(i)  ;
        }

        ROS_INFO_STREAM("Running command: " << command);

        execlp(args[0], args[0], args[1], args[2], nullptr);
        exit(EXIT_FAILURE);
    }
}

void SwathProcess::stop() {
    kill(m_swathRT_pid, SIGINT);
}