/*!
 * @file	Node.cpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#include <csignal>

#include "any_node/Node.hpp"
#include "message_logger/message_logger.hpp"

namespace any_node {

Node::Node(NodeHandlePtr nh):
    nh_(nh),
    workerManager_()
{
}

void Node::shutdown() {
    // raise SIGINT, which will be caught by the owner of the node instance and initiates the shutdown
    // todo: is there a better way?
    raise(SIGINT);
}

bool setProcessPriority(int priority) {
    sched_param params;
    params.sched_priority = priority;
    if(sched_setscheduler(getpid(), SCHED_FIFO, &params) != 0) {
        MELO_WARN("Failed to set process priority to %d: %s. Check /etc/security/limits.conf for the rights.", priority, std::strerror(errno));
        return false;
    }
    return true;
}


} // namespace any_node
