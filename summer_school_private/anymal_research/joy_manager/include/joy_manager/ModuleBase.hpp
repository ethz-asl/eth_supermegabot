/*!
* @file     ModuleBase.hpp
* @author   Linus Isler
* @date     May, 2016
* @brief
*/

#pragma once

#include <ros/ros.h>
#include <string>

#include "JoyManager.hpp"


namespace joy_manager {

class JoyManager;

class ModuleBase
{
 public:
  ModuleBase() {}
  virtual ~ModuleBase() {}
  virtual void init(const ros::NodeHandle& nh, 
                    joy_manager::JoyManager* joyManager, 
                    const std::string& name){
    ROS_DEBUG_STREAM("[ModuleBase] init() for " << name);
    nh_ = nh;
    joyManager_ = joyManager;
    name_ = name;
  };
  virtual void cleanup() {}
  virtual void processCommand(const std::string& command, std::string* answer) {}
  virtual void getName(std::string* name) {
    *name = name_;
  }
  virtual void setName(const std::string& name) {
    name_ = name;
  }

 protected:
  ros::NodeHandle nh_;
  JoyManager* joyManager_ = nullptr;
  std::string name_;

  void splitCommands(const std::string& command, std::vector<std::string>* commands) {
    std::size_t firstSpace = command.find(' ');
    if (firstSpace == std::string::npos){
      commands->push_back(command);
      commands->push_back("");
      commands->push_back("");
    }
    else {
      commands->push_back(command.substr(0, firstSpace));
      std::size_t secondSpace = command.find(' ', firstSpace + 1);
      if (secondSpace == std::string::npos) {
        commands->push_back(command.substr(firstSpace + 1, command.length()));
        commands->push_back("");
      }
      else {
        commands->push_back(command.substr(firstSpace + 1, secondSpace - (firstSpace + 1)));
        commands->push_back(command.substr(secondSpace + 1, command.length() - (secondSpace + 1)));
      }
    }
  }
};


} // namespace joy_manager
