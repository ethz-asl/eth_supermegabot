/*!
 * @file    ThrottledSubscriber.hpp
 * @author  Gabriel Hottiger
 * @date    Jan 19, 2017
 */

#pragma once

// STL
#include <functional>
#include <chrono>

// ros
#include <ros/ros.h>

// message logger
#include "message_logger/message_logger.hpp"

namespace any_node {

template <typename MessageType, typename CallbackClass>
class ThrottledSubscriber
{
public:
    ThrottledSubscriber()
    :   subscriber_(),
        fp_(nullptr),
        obj_(nullptr),
        lastTime_(),
        timeStep_()
    {

    }

    ThrottledSubscriber(const double timeStep, ros::NodeHandle& nh, const std::string& topic,
                        uint32_t queue_size, void(CallbackClass::*fp)(const boost::shared_ptr<MessageType const>&), CallbackClass* obj,
                        const ros::TransportHints& transport_hints = ros::TransportHints())
    :    fp_(fp),
        obj_(obj),
        lastTime_(ros::TIME_MIN),
        timeStep_(ros::Duration().fromSec(timeStep))
    {
      subscriber_ = nh.subscribe(topic, queue_size, &ThrottledSubscriber<MessageType, CallbackClass>::internalCallback, this, transport_hints);
    }

    virtual ~ThrottledSubscriber()
    {
        shutdown();
    }

    void shutdown() {
      subscriber_.shutdown();
    }

    void internalCallback(const boost::shared_ptr<MessageType const>& msg) {
      ros::Time now = ros::Time::now();
      if((now - lastTime_) >= timeStep_)
      {
        (*obj_.*fp_)(msg);
        lastTime_ = now;
      }
    }

protected:
    ros::Subscriber subscriber_;
    void(CallbackClass::*fp_)(const boost::shared_ptr<MessageType const>&);
    CallbackClass* obj_;
    ros::Time lastTime_;
    ros::Duration timeStep_;

};

template <typename MessageType, typename CallbackClass>
using ThrottledSubscriberPtr = std::shared_ptr<ThrottledSubscriber<MessageType, CallbackClass>>;


} // any_node
