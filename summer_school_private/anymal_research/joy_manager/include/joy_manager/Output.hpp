/*!
* @file     Output.hpp
* @author   Linus Isler
* @date     Jan, 2017
* @brief
*/

#pragma once

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace joy_manager {

enum JoyAxes {
  LATERAL=0,
  HEADING=1,
  ROLL=2,
  TURNING=3,
  VERTICAL=4,
  PITCH=5
};


class Output
{
 public:
  Output()
 : isPublishing_(true) {};
  virtual ~Output(){};
  virtual void init(const ros::NodeHandle& nh, 
                    const std::string& name, 
                    const std::string& topic, 
                    bool publish) {
    nh_ = nh;
    name_ = name;
    topic_ = topic;
    isPublishing_ = publish;
  };
  virtual void cleanup() = 0;
  virtual void publish(const sensor_msgs::Joy& msg) = 0;
  virtual void togglePublisher(bool publish) {
    isPublishing_ = publish;
  };

 protected:
  ros::NodeHandle nh_;
  std::string name_;
  std::string topic_;
  bool isPublishing_;
};

typedef std::shared_ptr<Output> OutputPtr;

} // namespace joy_manager
