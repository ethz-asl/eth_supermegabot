//
// Created by pleemann on 25.05.17.
//

//#define DEBUG_COSMO

#include "cosmo/cosmo.hpp"

#include "message_logger/message_logger.hpp"

#include "TestMessage.hpp"

#include <sys/time.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>


bool g_running = true;

void signalCallback(int) {
    g_running = false;
}

int main() { 
    sched_param params;
    params.sched_priority = 80;
    if(sched_setscheduler(getpid(), SCHED_FIFO, &params) != 0) {
        printf("Failed to set process priority: %s. Check /etc/security/limits.conf for the rights.", strerror(errno));
        return -1;
    }


    signal(SIGINT, signalCallback);
    cosmo::PublisherPtr<TestMessage> pub = cosmo::advertiseShm<TestMessage>("/test");
    TestMessage msg;

    for(int i=0; i<4; ++i) {
        msg.orientation[i] = double(i);
    }


        timespec ts;
    while(g_running) {
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);

        pub->publish(msg);

        // put cout after publish call to cancel effect of printout time difference
        std::cout << ts.tv_sec << "." << ts.tv_nsec << "\n";
        usleep(10000);
    }

    return true;
}
