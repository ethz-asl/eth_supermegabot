/*!
* @file     Feedback.hpp
* @author   Linus Isler
* @date     June 21st, 2016
* @brief
*/

#pragma once

#include <string>

#include <linux/input.h>
#include <ros/ros.h>

#include <any_node/any_node.hpp>
#include <sensor_msgs/JoyFeedbackArray.h>

namespace joy_interface {


class Feedback: public any_node::Node
{
 public:
  Feedback() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
  Feedback(any_node::Node::NodeHandlePtr nh);
  ~Feedback() override = default;
  bool update(const any_worker::WorkerEvent& event);
  bool init() override ;
  void cleanup() override {}

 protected:
  void joystickFeedbackCallback(const sensor_msgs::JoyFeedbackArray::ConstPtr& msg);
  std::string deviceName_;
  int fileDescriptor_;
  uint32_t features_[4];
  bool initializedJoystick_;
  bool isFirstFaultOpening_;
  std::vector<struct ff_effect> effects_;
  ros::Subscriber joystickFeedbackSubscriber_;

};

} // namespace joy_interface
