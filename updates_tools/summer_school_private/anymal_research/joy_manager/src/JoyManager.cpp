/*!
* @file     JoyManager.cpp
* @author   Linus Isler
* @date     May, 2016
*/

#include "joy_manager/JoyManager.hpp"

#include <string>

#include <std_msgs/Empty.h>

#include "joy_manager/JoyOutput.hpp"
#include "joy_manager/TwistOutput.hpp"
#include "joy_manager/PoseOutput.hpp"
#include "joy_manager/WrenchOutput.hpp"

namespace joy_manager {

JoyManager::JoyManager(any_node::Node::NodeHandlePtr nh)
 :
  any_node::Node(nh),
  moduleLoader_("joy_manager", "joy_manager::ModuleBase"),
  timeStep_(0.1),
  currentRequestedModules_(),
  currentRequestedCommands_(),
  priority_(0),
  hasReceivedEmergency_(false)
  {}


bool JoyManager::init() {
  ROS_DEBUG_STREAM("JoyManager init()");
  ros::NodeHandle nh = getNodeHandle();

  priority_ = 0;
  timer_ = ros::Duration(0.0);

  nh.getParam("time_step", timeStep_);

  std::vector<std::string> moduleNames;
  nh.getParam("modules", moduleNames);
  if (moduleNames.empty())
    ROS_WARN("No modules specified on parameter server!");
  double timerMaxDouble, timeOutDouble;
  nh.param<double>("timer", timerMaxDouble, 5.0);
  timerMax_ = ros::Duration(timerMaxDouble);
  nh.param<double>("timeout", timeOutDouble, 5.0);
  timeOut_ = ros::Duration(timeOutDouble);
  currentMessageStamp_ = ros::Time::now();

  // load the modules
  for (size_t i = 0; i < moduleNames.size(); i++) {
    try {
      modules_.push_back(moduleLoader_.createInstance(std::string("joy_manager::") + moduleNames[i]));
      modules_.back()->init(nh, this, moduleNames[i]);
    }
    catch(pluginlib::PluginlibException& ex) {
      ROS_WARN_STREAM("[" << moduleNames[i] << "] failed to load. Error: \n" << ex.what());
    }
  }

  // joysticks
  XmlRpc::XmlRpcValue params;
  nh.getParam("joysticks", params);
  for (int i = 0; i < params.size(); i++) {
    XmlRpc::XmlRpcValue param = params[i];
    std::string joystickName = static_cast<std::string>(param["name"]);
    std::string joystickTopic = static_cast<std::string>(param["topic"]);
    int joystickPriority = static_cast<int>(param["priority"]);
    ROS_DEBUG_STREAM("[" << joystickName << "](" << joystickPriority << ") " << joystickTopic);
    joysticks_.push_back(JoystickPtr(new joy_manager::Joystick(nh, this)));
    joysticks_.back()->setPriority(joystickPriority);
    joysticks_.back()->setName(joystickName);
    joysticks_.back()->setTopic(joystickTopic);
  }

  // publishers
  softEmergencyPublisher_ = any_node::advertise<std_msgs::Empty>(nh, "soft_emergency", "/soft_emcy_stop", 10, false);
  hardEmergencyPublisher_ = any_node::advertise<std_msgs::Empty>(nh, "hard_emergency", "/hard_emcy_stop", 10, false);
  notificationPublisher_.reset(new notification::NotificationPublisher("default", getNodeHandle(), true));

  XmlRpc::XmlRpcValue outputParams;
  nh.getParam("outputs", outputParams);
  for (int i = 0; i < outputParams.size(); i++) {
    XmlRpc::XmlRpcValue outputParam = outputParams[i];
    std::string outputName = static_cast<std::string>(outputParam["name"]);
    std::string outputTopic = static_cast<std::string>(outputParam["topic"]);
    std::string outputType = static_cast<std::string>(outputParam["type"]);
    bool outputIsPublishing = static_cast<bool>(outputParam["is_publishing"]);
    ROS_DEBUG_STREAM("[" << outputName << "](" 
                     << outputType << ") " 
                     << outputTopic << " (" 
                     << outputIsPublishing << ")");
    if (outputType == "Joy") {
      outputs_[outputName] = OutputPtr(new joy_manager::JoyOutput());
    }
    else if (outputType == "Pose") {
      outputs_[outputName] = OutputPtr(new joy_manager::PoseOutput());
    }
    else if (outputType == "Twist") {
      outputs_[outputName] = OutputPtr(new joy_manager::TwistOutput());
    }
    else if (outputType == "Wrench") {
      outputs_[outputName] = OutputPtr(new joy_manager::WrenchOutput());
    }
    else {
      ROS_WARN_STREAM("[JoyManager] Unsupported output: " << outputType);
      break;
    }
    outputs_[outputName]->init(nh, outputName, outputTopic, outputIsPublishing);
  }

  ros::Duration(0.5).sleep();
  notificationPublisher_->notify(notification::Level::LEVEL_INFO,
                                  std::string{"JM start up"},
                                  std::string{"Joy Manager is starting up."},
                                  "JoyManager");

  return addWorker(ros::this_node::getName() + "::updateWorker", param<double>("time_step", 1.0), &JoyManager::update, this, 90);
}


void JoyManager::cleanup() {
  notificationPublisher_->notify(notification::Level::LEVEL_ERROR,
                                 std::string{"JM shut down"},
                                 std::string{"Joy Manager is shutting down."},
                                 "JoyManager");
  ROS_DEBUG("Cleaning up Joy Manager.");
  for (ModulePtr modulePtr : modules_) {
    modulePtr->cleanup();
    modulePtr.reset();
  }
  for (auto& it : outputs_) {
    it.second->cleanup();
    it.second.reset();
  }
}


void JoyManager::checkPriority(const joy_manager_msgs::AnyJoy::ConstPtr& msg, int msgPriority) {
  boost::unique_lock<boost::shared_mutex> lockEmergency(emergencyMutex_);
  for (std::string command : msg->commands) {
    if (command == "soft_emcy_stop" || command == "hard_emcy_stop"){
      if (!hasReceivedEmergency_) {
        std::string joystickName;
        for (JoystickPtr joystick : joysticks_) {
          if (joystick->getPriority() == msgPriority) {
            joystickName = joystick->getName();
            break;
          }
        }
        ROS_WARN_STREAM("[JoyManager] " << command << " received by joystick [" << joystickName << "]!");
        std_msgs::Empty empty;
        std_msgs::EmptyConstPtr emptyMsg(new std_msgs::Empty(empty));
        if (command == "soft_emcy_stop")
          softEmergencyPublisher_.publish(emptyMsg);
        else if (command == "hard_emcy_stop")
          hardEmergencyPublisher_.publish(emptyMsg);
        notificationPublisher_->notify(notification::Level::LEVEL_ERROR,
                                       command,
                                       std::string{"Joy Manager has encountered "} + command,
                                       "JoyManager");
        hasReceivedEmergency_ = true;
      }
      return;
    }
  }
  if (hasReceivedEmergency_) {
    hasReceivedEmergency_ = false;
  }
  boost::unique_lock<boost::shared_mutex> lockPriority(priorityMutex_);
  if (msgPriority > priority_) {
    if (isNonZero(msg)) {
      priority_ = msgPriority;
      boost::unique_lock<boost::shared_mutex> lockTimer(timerMutex_);
      timer_ = timerMax_;
      boost::unique_lock<boost::shared_mutex> lock(joyMsgMutex_);
      currentJoyMsg_ = msg->joy;
      if (msg->modules.size() > 0) {
        currentRequestedModules_ = msg->modules;
        currentRequestedCommands_ = msg->commands;
      }
      currentMessageStamp_ = ros::Time::now();
    }
    else
      return;
  }
  else if (msgPriority == priority_) {
    if (isNonZero(msg)) {
      boost::unique_lock<boost::shared_mutex> lockTimer(timerMutex_);
      timer_ = timerMax_;
    }
    boost::unique_lock<boost::shared_mutex> lock(joyMsgMutex_);
    currentJoyMsg_ = msg->joy;
    if (msg->modules.size() > 0) {
      currentRequestedModules_ = msg->modules;
      currentRequestedCommands_ = msg->commands;
    }
    currentMessageStamp_ = ros::Time::now();
  }
}


void JoyManager::toggleOutput(const std::string& name, bool toggle) {
  auto it = outputs_.find(name);
  if (it != outputs_.end()) {
    it->second->togglePublisher(toggle);
  }
  else {
    ROS_WARN_STREAM("[JoyManager] " << name << " not found as an output.");
  }
}


bool JoyManager::update(const any_worker::WorkerEvent& event) {
  updateTimer();

  // read joystick axes, buttons and commands
  sensor_msgs::Joy nextJoyMsg;
  std::vector<std::string> nextRequestedCommands = {""};
  std::vector<std::string> nextRequestedModules = {""};
  {
    boost::unique_lock<boost::shared_mutex> lockJoyMsg(joyMsgMutex_);
    if (ros::Time::now() - currentMessageStamp_ < timeOut_) {
      nextJoyMsg = currentJoyMsg_;
    }
    nextRequestedModules = currentRequestedModules_;
    currentRequestedModules_ = {""};
    nextRequestedCommands = currentRequestedCommands_;
    currentRequestedCommands_ = {""};
  }

  // process commands and send notification back
  for (size_t i = 0; i < nextRequestedModules.size(); i++) {
    if ((nextRequestedModules[i] != "") && (nextRequestedCommands[i] != "")) {
      processCommand(nextRequestedModules[i], nextRequestedCommands[i]);
    }
  }
  
  // publish messages
  nextJoyMsg.header.stamp = ros::Time::now();
  for (auto& it : outputs_) {
    it.second->publish(nextJoyMsg);
  }
  return true;
}


void JoyManager::processCommand(const std::string& module, const std::string& command) {
  std::string moduleName = "";
  std::string answer = "unknown module";
  std::vector<std::string> outputDevice = {""};

  // find the right module
  ROS_DEBUG_STREAM("[JoyManager] command: " << command << " to: " << module);
  for (ModulePtr modulePtr : modules_) {
    modulePtr->getName(&moduleName);
    if (module == moduleName) {
      modulePtr->processCommand(command, &answer);
      break;
    }
  }
  if (answer != "") {
    boost::shared_lock<boost::shared_mutex> lockPriority(priorityMutex_);
    ROS_DEBUG_STREAM("[JoyManager] answer: " << answer);
    // find the currently top prioritized joystick
    for (JoystickPtr joystick : joysticks_) {
      if (joystick->getPriority() == priority_) {
        outputDevice[0] = joystick->getName();
        break;
      }
    }
    if (!outputDevice[0].empty()) {
      ros::Duration(0.2).sleep();
      notificationPublisher_->notify(notification::Level::LEVEL_INFO,
                                     answer,
                                     module + ": " + answer,
                                     "JoyManager",
                                     0);
    }
    else {
      ROS_WARN_STREAM("Couldn't send feedback to joystick! Joystick with priority " << priority_ << " not found.");
    }
  }
}


void JoyManager::updateTimer() {
  boost::unique_lock<boost::shared_mutex> lockTimer(timerMutex_);
  if (timer_.toSec() <= 0.0) {
    priority_ = 0;
    if (timer_.toSec() == 0.0) {
      timer_ = ros::Duration(-1.0);
    }
  }
  else {
    timer_ -= ros::Duration(timeStep_);
  }
}


bool JoyManager::isNonZero(const joy_manager_msgs::AnyJoy::ConstPtr& msg) {
  for (size_t i = 0; i < msg->joy.axes.size(); i++) {
    if (msg->joy.axes[i] != 0.0) {
      return true;
    }
  }
  for (size_t j = 0; j < msg->joy.buttons.size(); j++) {
    if (msg->joy.buttons[j] != 0) {
      return true;
    }
  }
  if (!msg->modules.empty() && msg->modules[0] != "") {
    return true;
  }
  return false;
}

} // namespace joy_manager
