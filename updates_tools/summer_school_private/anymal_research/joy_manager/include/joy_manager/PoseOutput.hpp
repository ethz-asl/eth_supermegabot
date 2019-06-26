/*!
* @file     PoseOutput.hpp
* @author   Linus Isler
* @date     Jan, 2017
* @brief
*/

#pragma once


#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <kindr/Core>
#include <cosmo_ros/cosmo_ros.hpp>
#include <any_measurements/Pose.hpp>
#include <any_measurements_ros/ConversionTraits.hpp>

#include "joy_manager/Output.hpp"


namespace joy_manager {

class PoseOutput : public joy_manager::Output
{
 public:
  using PoseShm = any_measurements::Pose;
  using PoseRos = geometry_msgs::PoseStamped;

  PoseOutput();
  virtual ~PoseOutput(){};
  virtual void init(const ros::NodeHandle& nh,
                    const std::string& name,
                    const std::string& topic,
                    bool publish);
  virtual void cleanup();
  virtual void publish(const sensor_msgs::Joy& msg);

 protected:
  void poseMinCallback(const PoseRos::ConstPtr& msg);
  void poseMaxCallback(const PoseRos::ConstPtr& msg);
  // calculate the pose from the joy values and publish them
  void mapJoyToPose(const sensor_msgs::Joy& msg);
  void loadParams(const ros::NodeHandle& nh);

  // ros::Publisher posePublisher_;
  cosmo_ros::PublisherRosPtr<PoseShm, PoseRos, any_measurements_ros::ConversionTraits> posePublisher_;

  ros::Subscriber poseMinSubscriber_;
  kindr::HomTransformQuatD poseMin_;
  boost::shared_mutex poseMinMutex_;
  ros::Subscriber poseMaxSubscriber_;
  kindr::HomTransformQuatD poseMax_;
  boost::shared_mutex poseMaxMutex_;
  std::string poseMinTopic_;
  std::string poseMaxTopic_;
  std::string frameIdName_;

};

} // namespace joy_manager
