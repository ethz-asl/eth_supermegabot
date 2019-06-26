/*!
 * @file 	  SmbJoystickController.cpp
 * @author   Johannes Pankert
 * @date		  27/11/2018
 * @version 	1.0
 * @brief    A controller that ...
 */

// smb_joystick_controller
#include "smb_joystick_controller/SmbJoystickController.hpp"

namespace smb_joystick_controller {

SmbJoystickController::SmbJoystickController() : Base() { setName("SmbJoystickController"); }

SmbJoystickController::~SmbJoystickController() {}

bool SmbJoystickController::create(double dt) { return true; }

bool SmbJoystickController::initialize(double dt) {
  // load parameters
  std::string parameter_path = this->getParameterPath();
  MELO_INFO_STREAM("SmbJoystickController parameter_path: " << parameter_path);
  controlParams_ = yaml_tools::YamlNode::fromFile(parameter_path);
  if (controlParams_.isNull()) {
    MELO_ERROR("[SmbJoystickController::loadParameters]: Could not load parameter file %s!", parameter_path.c_str());
    return false;
  }

  std::string topicName = controlParams_["subscribers"]["joy_twist"]["topic"].as<std::string>();
  int queueSize = controlParams_["subscribers"]["joy_twist"]["queue_size"].as<int>();

  joystickTwistSubscriber_ =
      getNodeHandle().subscribe(topicName, queueSize, &SmbJoystickController::twistCallback, this, ros::TransportHints().tcpNoDelay());

  MELO_INFO_STREAM("Controller " << this->getName() << " is initialized!");

  return true;
}

bool SmbJoystickController::advance(double dt) {
  std::lock_guard<std::mutex> lockGuard(twistMutex_);
  if (ros::Time::now().toSec() - joyTwist_.time_.toSeconds() < 1.0) {
    boost::lock_guard<boost::shared_mutex> commandLockGuard(getCommandMutex());
    getCommand().setBaseTwistCommand(joyTwist_);
  } else {
    boost::lock_guard<boost::shared_mutex> commandLockGuard(getCommandMutex());
    getCommand().freeze();
    MELO_WARN_THROTTLE_STREAM(2, "[SmbJoystickController::advance] twist command age: "
                                     << joyTwist_.time_.toSeconds() << "s. Send freeze command. Now: " << ros::Time::now());
  }
  return true;
}

bool SmbJoystickController::reset(double dt) { return SmbJoystickController::initialize(dt); }

bool SmbJoystickController::preStop() {
  joystickTwistSubscriber_.shutdown();
  return true;
}

bool SmbJoystickController::stop() { return true; }

bool SmbJoystickController::cleanup() { return true; }

void SmbJoystickController::twistCallback(const geometry_msgs::TwistStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lockGuard(twistMutex_);
  any_measurements_ros::fromRos(*msg, joyTwist_);
}

} /* namespace smb_joystick_controller */
