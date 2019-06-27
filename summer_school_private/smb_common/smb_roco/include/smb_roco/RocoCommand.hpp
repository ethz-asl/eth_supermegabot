/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Christian Gehring
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * @file     RocoCommand.hpp
 * @author   Johannes Pankert
 * @date     June, 2019
 */

#pragma once

// roco
#include <roco/model/CommandInterface.hpp>

// stl
#include <ostream>

// any_measurements
#include <any_measurements/Time.hpp>
#include <any_measurements/Twist.hpp>
#include <any_measurements_ros/any_measurements_ros.hpp>

// smb_description
#include <smb_description/SmbDescription.hpp>

// smb_common
#include <smb_common/SmbCommands.hpp>

// util
#include <message_logger/message_logger.hpp>

namespace smb_roco {

class RocoCommand : public roco::CommandInterface {
 public:
  using SmbCommandsShm = smb_common::SmbCommands<smb_description::SmbDescription>;

  RocoCommand();
  virtual ~RocoCommand();

  /*! Limits the command
   * @returns false if there is a NaN or Inf.
   */
  bool limitCommand();
  friend std::ostream& operator<<(std::ostream& out, const RocoCommand& command);

  void setBaseTwistCommand(const any_measurements::Twist& baseTwistCommand);

  void freeze();

  const SmbCommandsShm& getActuatorCommands();

  void setMaxLinearVelocity(double maxLinearVelocity);

  void setMaxRotationalVelocity(double maxRotationalVelocity);

  void setMaxLinearAcceleration(double maxLinearAcceleration);

  void setMaxRotationalAcceleration(double maxRotationalAcceleration);

  void setDt(double dt);

 protected:
  void skidSteerConversion(const any_measurements::Twist& twist, double& leftWheelVel, double& rightWheelVel);
  void applyLimits(double& forwardVelocity, double& rotationalVelocity);

 protected:
  SmbCommandsShm actuatorCommands_;

  // smb dimensions for skid steer controller. Should we read them from the urdf file?
  static constexpr double wheelOffset_ = 0.32;  //[m]
  static constexpr double wheelRadius_ = 0.19;  //[m]

  double maxLinearVelocity_ = 2.0;
  double maxRotationalVelocity_ = 2.0;
  double maxLinearAcceleration_ = 5.0;
  double maxRotationalAcceleration_ = 10.0;
  double dt_ = 0.02;

  double lastForwardVelocity_ = 0.0;
  double lastRotationalVelocity_ = 0.0;
};

} /* namespace smb_roco */