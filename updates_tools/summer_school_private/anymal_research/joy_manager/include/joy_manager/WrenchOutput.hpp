/*!
* @file     WrenchOutput.hpp
* @author   Koen Krämer
* @date     Nov, 2017
* @brief
*/

#pragma once

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kindr/Core>
#include <cosmo_ros/cosmo_ros.hpp>
#include <any_measurements/Twist.hpp>
#include <any_measurements_ros/ConversionTraits.hpp>

#include "joy_manager/Output.hpp"

namespace joy_manager {

    class WrenchOutput : public joy_manager::Output
    {
    public:
        using WrenchShm = any_measurements::Wrench;
        using WrenchRos = geometry_msgs::WrenchStamped;

        WrenchOutput();
        virtual ~WrenchOutput(){};
        virtual void init(const ros::NodeHandle& nh,
                          const std::string& name,
                          const std::string& topic,
                          bool publish);
        virtual void cleanup();
        virtual void publish(const sensor_msgs::Joy& msg);

    protected:
        void wrenchMinCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
        void wrenchMaxCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
        // calculate the wrench from the joy values and publish them
        void mapJoyToWrench(const sensor_msgs::Joy& msg);
        void loadParams(const ros::NodeHandle& nh);

        cosmo_ros::PublisherRosPtr<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits> wrenchPublisher_;
        ros::Subscriber wrenchMinSubscriber_;
        kindr::WrenchD wrenchMin_;
        boost::shared_mutex wrenchMinMutex_;
        ros::Subscriber wrenchMaxSubscriber_;
        kindr::WrenchD wrenchMax_;
        boost::shared_mutex wrenchMaxMutex_;
        std::string wrenchMinTopic_;
        std::string wrenchMaxTopic_;
        std::string frameIdName_;
    };

} // namespace joy_manager
