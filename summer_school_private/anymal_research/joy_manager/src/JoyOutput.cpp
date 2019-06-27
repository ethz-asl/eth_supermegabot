/*!
* @file     JoyOutput.cpp
* @author   Linus Isler
* @date     Jan, 2017
* @brief
*/

#include <kindr_ros/kindr_ros.hpp>

#include "joy_manager/JoyOutput.hpp"

namespace joy_manager {


JoyOutput::JoyOutput()
{}


void JoyOutput::init(const ros::NodeHandle& nh, 
                     const std::string& name, 
                     const std::string& topic, 
                     bool publish) {
  Output::init(nh, name, topic, publish);

  joyPublisher_ = nh_.advertise<sensor_msgs::Joy>(topic, 100);
}


void JoyOutput::cleanup() {
  
}


void JoyOutput::publish(const sensor_msgs::Joy& msg) {
  if (isPublishing_) {
    if (joyPublisher_.getNumSubscribers() > 0u) {
      sensor_msgs::JoyConstPtr joyMsg(new sensor_msgs::Joy(msg));
      joyPublisher_.publish(joyMsg);
    }  
  }
}

} // namespace joy_manager
