/*!
* @file     JoyOutput.hpp
* @author   Linus Isler
* @date     Jan, 2017
* @brief
*/

#pragma once


#include <string>

#include <ros/ros.h>

#include <joy_manager/Output.hpp>


namespace joy_manager {

class JoyOutput : public joy_manager::Output
{
 public:
  JoyOutput();
  virtual ~JoyOutput(){};
  virtual void init(const ros::NodeHandle& nh, 
                    const std::string& name, 
                    const std::string& topic, 
                    bool publish);
  virtual void cleanup();
  virtual void publish(const sensor_msgs::Joy& msg);

 protected:
  ros::Publisher joyPublisher_;
};

} // namespace joy_manager
