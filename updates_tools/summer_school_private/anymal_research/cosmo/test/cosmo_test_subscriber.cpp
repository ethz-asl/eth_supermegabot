//
// Created by pleemann on 25.05.17.
//

//#define DEBUG_COSMO

#include "cosmo/cosmo.hpp"

#include "message_logger/message_logger.hpp"

#include "TestMessage.hpp"

#include <sys/time.h>
#include <csignal>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>

bool g_running = true;

void signalCallback(int) {
    g_running = false;
}

class myClass {
public:
    void callback(const TestMessage& msg) {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
        std::cout << ts.tv_sec << "." << ts.tv_nsec << "\n"; 
    }
};

int main() {
    sched_param params;
    params.sched_priority = 90;
    if(sched_setscheduler(getpid(), SCHED_FIFO, &params) != 0) {
        printf("Failed to set process priority: %s. Check /etc/security/limits.conf for the rights.", strerror(errno));
        return -1;
    }

    signal(SIGINT, signalCallback);
    myClass instance;
    cosmo::SubscriberPtr<TestMessage> sub = cosmo::subscribeShm("/test", &myClass::callback, &instance);
    while(g_running) {
        sub->receive(std::chrono::microseconds{1000000});
    }

    return true;
}
