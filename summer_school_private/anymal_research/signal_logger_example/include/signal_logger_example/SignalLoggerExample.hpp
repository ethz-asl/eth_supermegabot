/*
 * SignalLoggerExample.hpp
 *
 *  Created on: May 18, 2017
 *      Author: Gabriel Hottiger
 */
#pragma once

// nodewrap
#include "any_node/Node.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

// ros
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

// Boost
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// STL
#include <atomic>
#include <thread>
#include <vector>
#include <array>
#include <map>
#include <set>
#include <unordered_map>
#include <list>
#include <signal.h>
#include <stdlib.h>

namespace signal_logger_example {

class SignalLoggerExample: public any_node::Node
{
  public:
    SignalLoggerExample(NodeHandlePtr nh);

    bool init() override;
    void cleanup() override;
    virtual bool update(const any_worker::WorkerEvent& event);
    virtual void publishWorker();
    virtual void readWorker();

  private:
    std::thread publishThread_;
    std::thread readThread_;
    std::atomic_bool shouldPublish_;
    std::atomic_bool shouldRead_;
    double logVar_;
    ros::Time time_;
    // STL
    std::string string_;
    std::vector<double> vec_;
    std::array<int, 3> arr_;
    std::list<float> list_;
    std::set<long> set_;
    std::deque<short> deque_;
    std::pair<std::string, double> pair_;
    std::map<std::string, int> map_;
    std::unordered_map<int, double> umap_;

};

} /* namespace m545_tf_publisher */
