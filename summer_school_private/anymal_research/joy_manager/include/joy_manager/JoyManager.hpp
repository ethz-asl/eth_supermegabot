/*!
* @file     JoyManager.hpp
* @author   Linus Isler
* @date     May, 2016
*/

#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>

#include <any_node/any_node.hpp>
#include <boost/thread.hpp>
#include <notification/NotificationPublisher.hpp>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/Joy.h>

#include <joy_manager_msgs/AnyJoy.h>
#include "joy_manager/Joystick.hpp"
#include "joy_manager/ModuleBase.hpp"
#include "joy_manager/Output.hpp"


namespace joy_manager {

class Joystick;

typedef boost::shared_ptr<joy_manager::Joystick> JoystickPtr;

class ModuleBase;

typedef boost::shared_ptr<ModuleBase> ModulePtr;

class JoyManager: public any_node::Node
{
 public:
  JoyManager() = delete;  // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
  JoyManager(any_node::Node::NodeHandlePtr nh);
  ~JoyManager() override = default;
  virtual bool update(const any_worker::WorkerEvent& event);
  bool init() override;
  void cleanup() override;

  /*!
   * @brief Check if priority of an AnyJoy-msg from a Joystick is high enough to further process
   * @param msg          Incomming AnyJoy msg from a joystick interface.
   * @param msgPriority  Its priority, normally a member of the Joystick object.
   */
  void checkPriority(const joy_manager_msgs::AnyJoy::ConstPtr& msg, int msgPriority);
  
  /*!
   * Toggle a specific output to be silenced or not.
   * @param name Name of the publisher can be read from rosparam.
   * @param toggle True if the topic should be published.
   */
  void toggleOutput(const std::string& name, bool toggle);

 protected:
  // find requested module, process commands and publish notification
  void processCommand(const std::string& module, const std::string& command);
  // update the timer
  void updateTimer();
  // check if a AnyJoy-msg has content
  bool isNonZero(const joy_manager_msgs::AnyJoy::ConstPtr& msg);

  // plugin modules
  pluginlib::ClassLoader<joy_manager::ModuleBase> moduleLoader_;
  std::vector<ModulePtr> modules_;

  // subscriber and mutexes
  std::vector<JoystickPtr> joysticks_;

  // publisher and worker
  ros::Publisher softEmergencyPublisher_;
  ros::Publisher hardEmergencyPublisher_;
  std::shared_ptr<notification::NotificationPublisher> notificationPublisher_;
  std::map<std::string, OutputPtr> outputs_;
  
  double timeStep_;

  sensor_msgs::Joy currentJoyMsg_;
  std::vector<std::string> currentRequestedModules_;
  std::vector<std::string> currentRequestedCommands_;
  boost::shared_mutex joyMsgMutex_;
  ros::Duration timer_;
  ros::Duration timerMax_;
  ros::Duration timeOut_;
  ros::Time currentMessageStamp_;
  boost::shared_mutex timerMutex_;
  int priority_;
  boost::shared_mutex priorityMutex_;
  bool hasReceivedEmergency_;
  boost::shared_mutex emergencyMutex_;

};

} // namespace joy_manager
