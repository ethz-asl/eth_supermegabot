/*!
* @file     JoyInterface.hpp
* @author   Linus Isler
* @date     October 13th, 2016
* @brief
*/

#pragma once

#include <vector>

#include <ros/ros.h>

#include <any_node/any_node.hpp>
#include <joy_manager_msgs/AnyJoy.h>
#include <notification/NotificationSubscriber.hpp>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>

#include "InterfaceModuleBase.hpp"


namespace joy_interface {


class JoyInterface: public any_node::Node
{
 public:
  JoyInterface() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
  JoyInterface(any_node::Node::NodeHandlePtr nh);
  ~JoyInterface() override = default;
  virtual bool update(const any_worker::WorkerEvent& event) {return true;};
  bool init() override;
  void cleanup() override;

 protected:
  // listener to the joystick topic
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);
  // notifications to the joystick will be handled here
  void notificationCallback(const notification::Notification& msg);

  // Subscribers
  ros::Subscriber joystickSubscriber_;
  boost::shared_ptr<notification::NotificationSubscriber> notificationSubscriber_;

  // Publishers
  ros::Publisher anyJoyPublisher_;
  ros::Publisher joyFeedbackPublisher_;

  // Plugin modules
  pluginlib::ClassLoader<joy_interface::InterfaceModuleBase> moduleLoader_;
  std::vector<InterfaceModulePtr> modules_;

  // Parameters from config file
  std::string deviceName_;
  std::vector<int> axisMapping_;
  std::vector<int> buttonMapping_;
  int emergencyButtonIndex_;
  bool sendHardEmergencyStop_;

  bool sendFeedback_;
  int feedbackThreshold_;
  float feedbackIntensity_;
};

} // namespace joy_interface
