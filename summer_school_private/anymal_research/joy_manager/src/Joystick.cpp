/*!
* @file     Joystick.cpp
* @author   Linus Isler
* @date     May, 2016
* @brief
*/

#include <joy_manager/Joystick.hpp>



namespace joy_manager {


Joystick::Joystick(const ros::NodeHandle& nh, joy_manager::JoyManager* joyManager)
:
  nh_(nh),
  joyManager_(joyManager),
  priority_(0),
  name_("")
{}


Joystick::~Joystick() {
}


void Joystick::setName(const std::string& name) {
  name_ = name;
}


const std::string Joystick::getName() const {
  return name_;
}


void Joystick::setPriority(const int& priority) {
  priority_ = priority;
}


int Joystick::getPriority() const {
  return priority_;
}


void Joystick::setTopic(const std::string topic) {
  anyJoySubscriber_ = nh_.subscribe(topic, 10, &Joystick::callback, this);
}


const std::string Joystick::getTopic() const {
  return anyJoySubscriber_.getTopic();
}

int Joystick::getNumberOfPublishers() const {
  return anyJoySubscriber_.getNumPublishers();
}


void Joystick::callback(const joy_manager_msgs::AnyJoy::ConstPtr& msg) {
  joyManager_->checkPriority(msg, priority_);
}


} // namespace joy_manager
