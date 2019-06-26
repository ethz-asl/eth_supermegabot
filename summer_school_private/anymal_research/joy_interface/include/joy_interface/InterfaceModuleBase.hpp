/*!
* @file     InterfaceModuleBase.hpp
* @author   Linus Isler
* @date     October 13th, 2016
* @brief
*/

#pragma once

#include <string>

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

namespace joy_interface {

class InterfaceModuleBase
{
 public:
  InterfaceModuleBase(){
    name_ = "";
    deviceName_ = "";
  };
  virtual ~InterfaceModuleBase(){};
  virtual void init(const ros::NodeHandle& nh, const std::string& name){
    ROS_DEBUG_STREAM("[InterfaceModuleBase] init() for " << name);
    nh_ = nh;
    name_ = name;
  };
  virtual void cleanup(){};
  virtual void processJoy(const sensor_msgs::Joy::ConstPtr& joy,
                          std::vector<std::string>* modules,
                          std::vector<std::string>* commands){};
  virtual std::string getName() {
    return name_;
  }
  virtual void setDevice(const std::string& deviceName) {
    deviceName_ = deviceName;
  }
  virtual void setName(const std::string& name) {
    name_ = name;
  }

 protected:
  ros::NodeHandle nh_;
  std::string name_;
  std::string deviceName_;
};

typedef boost::shared_ptr<InterfaceModuleBase> InterfaceModulePtr;

} // namespace joy_interface
