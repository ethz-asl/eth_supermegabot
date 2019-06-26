/*!
 * @file	SubscriberRosOptions.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/SubscriberOptions.hpp"

#include <ros/transport_hints.h>
#include <ros/node_handle.h>

namespace cosmo_ros {

template<typename Msg_>
class SubscriberRosOptions : public cosmo::SubscriberOptions<Msg_>
{
public:
    using BaseType = cosmo::SubscriberOptions<Msg_>;
    using SubscriberCallback = typename BaseType::SubscriberCallback;

    /**
     *
     * @param[in]  topic       ROS topic name and identifier of the shared
     *                         memory.
     * @param[in]  callback    Shared memory subscriber callback
     * @param[in]  nodeHandle  ROS nodehandle
     */
    SubscriberRosOptions(const std::string& topic,
                         const SubscriberCallback& callback,
                         const ros::NodeHandle& nodeHandle = ros::NodeHandle()):
            BaseType(topic, callback),
            nodeHandle_(nodeHandle),
            rosTransportHints_(),
            rosQueueSize_(100),
            tryRosResubscribing_(true)
    {
    }

    ~SubscriberRosOptions() override = default;

public:
    mutable ros::NodeHandle nodeHandle_;
    ros::TransportHints rosTransportHints_;

    unsigned int rosQueueSize_;

    //! if set to true, reconnect to the ros subscriber if the waitTime_ on the shared memory was exceeded
    bool tryRosResubscribing_;
};

template<typename Msg_>
using SubscriberRosOptionsPtr = std::shared_ptr<SubscriberRosOptions<Msg_>>;

} // namespace 
