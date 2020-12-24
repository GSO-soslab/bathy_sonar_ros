#include <poll.h>
#include <csignal>
#include "swath_proc.h"
#include "unistd.h"

using namespace soslab;

void SwathProcess::start() {

    m_swathRT_pid = fork();
    if(m_swathRT_pid == 0) {
        execlp("swathRT",
               m_config_path.empty() ? "" : "-s",
               m_config_path.empty() ? "" : m_config_path.c_str(),
               nullptr);
        exit(EXIT_FAILURE);
    }
}

void SwathProcess::stop() {
    kill(m_swathRT_pid, SIGINT);
}