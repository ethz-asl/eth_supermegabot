/*!
* @file     JoyInterface.cpp
* @author   Linus Isler
* @date     October 13th, 2016
* @brief
*/


#include "joy_interface/JoyInterface.hpp"
#include <joy_manager_msgs/AnyJoy.h>


namespace joy_interface {

JoyInterface::JoyInterface(any_node::Node::NodeHandlePtr nh)
  : any_node::Node(nh),
    moduleLoader_("joy_interface", "joy_interface::InterfaceModuleBase"),
    emergencyButtonIndex_(3),
    sendHardEmergencyStop_(false),
    sendFeedback_(false),
    feedbackThreshold_(0),
    feedbackIntensity_(0.0)
  {
    axisMapping_ = {0, 1, 2, 3, 4, 5};
    buttonMapping_ = {0, 1, 2, 3, 4, 5, 6, 7};
  }

bool JoyInterface::init() {
  ros::NodeHandle nh = getNodeHandle();

  nh.param<std::string>("device", deviceName_, "device");
  nh.param<bool>("feedback", sendFeedback_, false);

  std::vector<std::string> moduleNames;
  nh.getParam("modules", moduleNames);

  nh.param<bool>("hardEmergencyStop", sendHardEmergencyStop_, false);
  nh.param<int>("emergencyButtonIndex", emergencyButtonIndex_, 3);
  nh.getParam("axisMapping", axisMapping_);
  nh.getParam("buttonMapping", buttonMapping_);

  // load the modules
  for (size_t i = 0; i < moduleNames.size(); i++) {
    try {
      modules_.push_back(moduleLoader_.createInstance(std::string("joy_interface::") + moduleNames[i]));
      modules_.back()->init(nh, moduleNames[i]);
      modules_.back()->setDevice(deviceName_);
    }
    catch(pluginlib::PluginlibException& ex) {
      ROS_WARN_STREAM("[" << moduleNames[i] << "] failed to load. Error: \n" << ex.what());
    }
  }

  // subscriber & publisher
  joystickSubscriber_
    = any_node::subscribe(nh, "joy", "/joy", 10, &JoyInterface::joystickCallback, this);
  anyJoyPublisher_
    = any_node::advertise<joy_manager_msgs::AnyJoy>(nh, "anyJoy", "/anyjoy", 10, false);

  if (sendFeedback_) {
    nh.param<float>("feedbackIntensity", feedbackIntensity_, 0.5);
    nh.param<int>("notification/level/vibration", feedbackThreshold_, 3);
    notificationSubscriber_.reset(new notification::NotificationSubscriber(
        "operator_screen", getNodeHandle(), 0, 20,
        boost::bind(&JoyInterface::notificationCallback, this, _1)));
    joyFeedbackPublisher_
      = any_node::advertise<sensor_msgs::JoyFeedbackArray>(nh, "feedback", "/feedback", 10, false);
  }

  return addWorker(ros::this_node::getName() + "::updateWorker", param<double>("time_step", 1.0), &JoyInterface::update, this, 90);
}


void JoyInterface::cleanup(){
  for (InterfaceModulePtr modulePtr : modules_) {
    modulePtr->cleanup();
    modulePtr.reset();
  }
}


void JoyInterface::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  joy_manager_msgs::AnyJoy anyJoy;
  anyJoy.joy.axes.assign(6, 0.0);
  anyJoy.joy.buttons.assign(8, 0);
  anyJoy.header.stamp = ros::Time::now();

  // interpret joystick buttons:
  // emergency stop button (specified in config file)
  if (msg->buttons[emergencyButtonIndex_] == 1) {
    if (sendHardEmergencyStop_) {
      anyJoy.commands.push_back("hard_emcy_stop");
    }
    else {
      anyJoy.commands.push_back("soft_emcy_stop");
    }
  }

  // send axes and buttons to interface modules
  std::string moduleName;
  for (InterfaceModulePtr modulePtr : modules_) {
    modulePtr->processJoy(msg, &anyJoy.modules, &anyJoy.commands);
  }

  // map the axes and buttons from joy to anyjoy message
  anyJoy.joy.header = msg->header;
  for (size_t i = 0; i < msg->axes.size(); i++) {
    if (axisMapping_[i] < 0) continue;

    int axis = axisMapping_[i];
    float axisValue = msg->axes[i];
    if (axisValue != axisValue) { // NaN check
      anyJoy.commands.push_back("soft_emcy_stop");
      break;
    }
    anyJoy.joy.axes[axis] = axisValue;
  }
  for (int i = 0; i < 8; i++) {
    if (buttonMapping_[i] < 0) {
      anyJoy.joy.buttons[i] = 0;
    }
    else {
      int buttonValue = msg->buttons[buttonMapping_[i]];
      if (buttonValue != 0 && buttonValue != 1) {
        anyJoy.commands.push_back("soft_emcy_stop");
        break;
      }
      anyJoy.joy.buttons[i] = buttonValue;
    }
  }
  anyJoyPublisher_.publish(anyJoy);
}


void JoyInterface::notificationCallback(const notification::Notification& msg) {
  if (msg.getLevelAsUnsignedChar() >= feedbackThreshold_) {
    sensor_msgs::JoyFeedbackArray feedbackMsg;
    sensor_msgs::JoyFeedback feedback;
    feedback.id = 0;
    feedback.intensity = feedbackIntensity_;
    feedback.type = feedback.TYPE_RUMBLE;
    feedbackMsg.array.push_back(feedback);
    joyFeedbackPublisher_.publish(feedbackMsg);
  }
}


} // namespace joy_interface
