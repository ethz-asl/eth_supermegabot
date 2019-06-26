/*!
* @file 	  SmbZeroGravityController.hpp
* @author   Johannes Pankert
* @date		  27/11/2018
* @version 	1.0
* @brief    A controller that ...
*/

#pragma once

// roco_ros
#include "roco_ros/controllers/controllers.hpp"
#include <roco_ros/controllers/ControllerRos.hpp>

// eigen
#include <Eigen/Core>
#include <Eigen/SVD>

// state and command
#include "smb_roco/RocoState.hpp"
#include "smb_roco/RocoCommand.hpp"

// utils
#include <yaml_tools/YamlNode.hpp>

//math
#include <cmath>

//ros
#include <geometry_msgs/TwistStamped.h>

// any...
#include <any_measurements/Twist.hpp>
#include <any_measurements_ros/ConversionTraits.hpp>

namespace smb_joystick_controller {

class SmbJoystickController: virtual public roco_ros::ControllerRos<smb_roco::RocoState, smb_roco::RocoCommand> {

 public:
  using Base = roco_ros::ControllerRos<smb_roco::RocoState, smb_roco::RocoCommand>;

  SmbJoystickController();

  virtual ~SmbJoystickController();

 protected:
  virtual bool create(double dt);
  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  virtual bool reset(double dt);
  virtual bool preStop();
  virtual bool stop();
  virtual bool cleanup();

  private:
    // Parameter parsing object
    yaml_tools::YamlNode controlParams_;
    ros::Subscriber joystickTwistSubscriber_;

    any_measurements::Twist joyTwist_;
    std::mutex twistMutex_;

    void twistCallback(const geometry_msgs::TwistStampedConstPtr& msg);

  };

} /* namespace smb_joystick_controller */
