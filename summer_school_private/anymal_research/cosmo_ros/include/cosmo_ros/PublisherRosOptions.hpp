/*!
 * @file	PublisherRosOptions.hpp
 * @author	Philipp Leemann, Christian Gehring
 * @date	May 24, 2017
 */

#pragma once

#include "cosmo/PublisherOptions.hpp"

#include <ros/node_handle.h>

namespace cosmo_ros {

//! Options for the ROS publisher
class PublisherRosOptions : public cosmo::PublisherOptions
{
public:

    /**
     *
     * @param[in]  topic       ROS topic name and identifier of the shared
     *                         memory.
     * @param[in]  nodeHandle  ROS node handle
     */
    PublisherRosOptions(const std::string& topic, const ros::NodeHandle& nodeHandle = ros::NodeHandle()):
            PublisherOptions(topic),
            nodeHandle_(nodeHandle),
            autoPublishRos_{false},
            rosQueueSize_{100},
            rosLatch_{false}
    {
    }

    virtual ~PublisherRosOptions() {}
public:
    //! Node handle
    mutable ros::NodeHandle nodeHandle_;

    //! If true, the publisher will publish the ROS messages automatically in another thread.
    bool autoPublishRos_;

    //! Length of the ROS queue
    unsigned int rosQueueSize_;

    //! Wheter ROS messages should be latched
    bool rosLatch_;
};


using PublisherRosOptionsPtr = std::shared_ptr<PublisherRosOptions>;

} // namespace 
