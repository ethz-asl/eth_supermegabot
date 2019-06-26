/*!
* @file     Joystick.hpp
* @author   Linus Isler
* @date     May, 2016
* @brief
*/

#pragma once

#include <ros/ros.h>
#include <string>

#include <boost/thread.hpp>

#include <joy_manager_msgs/AnyJoy.h>
#include <joy_manager/JoyManager.hpp>
#include "ModuleBase.hpp"


namespace joy_manager {

class JoyManager;

class Joystick {
 public:
  Joystick(const ros::NodeHandle& nh, joy_manager::JoyManager* joyManager);
  virtual ~Joystick();
  void setName(const std::string& name);
  const std::string getName() const;
  void setPriority(const int& priority);
  int getPriority() const;
  void setTopic(const std::string topic);
  const std::string getTopic() const;
  int getNumberOfPublishers() const;

 protected:
  ros::NodeHandle nh_;
  JoyManager* joyManager_;
  int priority_;
  std::string name_;
  ros::Subscriber anyJoySubscriber_;
  void callback(const joy_manager_msgs::AnyJoy::ConstPtr& msg);

};



} // namespace joy_manager
