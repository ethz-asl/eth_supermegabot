/*!
* @file     TwistOutput.hpp
* @author   Linus Isler
* @date     Jan, 2017
* @brief
*/

#pragma once


#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <kindr/Core>
#include <cosmo_ros/cosmo_ros.hpp>
#include <any_measurements/Twist.hpp>
#include <any_measurements_ros/ConversionTraits.hpp>

#include "joy_manager/Output.hpp"


namespace joy_manager {

class TwistOutput : public joy_manager::Output
{
 public:
  using TwistShm = any_measurements::Twist;
  using TwistRos = geometry_msgs::TwistStamped;

  TwistOutput();
  virtual ~TwistOutput(){};
  virtual void init(const ros::NodeHandle& nh,
                    const std::string& name,
                    const std::string& topic,
                    bool publish);
  virtual void cleanup();
  virtual void publish(const sensor_msgs::Joy& msg);

 protected:
  void twistMinCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void twistMaxCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  // calculate the twist from the joy values and publish them
  void mapJoyToTwist(const sensor_msgs::Joy& msg);
  void loadParams(const ros::NodeHandle& nh);

  // ros::Publisher twistPublisher_;
  cosmo_ros::PublisherRosPtr<TwistShm, TwistRos, any_measurements_ros::ConversionTraits> twistPublisher_;
  ros::Subscriber twistMinSubscriber_;
  kindr::TwistLocalD twistMin_;
  boost::shared_mutex twistMinMutex_;
  ros::Subscriber twistMaxSubscriber_;
  kindr::TwistLocalD twistMax_;
  boost::shared_mutex twistMaxMutex_;
  std::string twistMinTopic_;
  std::string twistMaxTopic_;
  std::string frameIdName_;

};

} // namespace joy_manager
