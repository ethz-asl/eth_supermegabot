/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Christian Gehring, Dario Bellicoso
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
 * @file     Command.cpp
 * @author   Johannes Pankert
 * @date     June, 2019
 */

// smb roco
#include "smb_roco/RocoCommand.hpp"

// message logger
#include <message_logger/message_logger.hpp>

// stl
#include <cmath>

#include <romo_std/common/container_utils.hpp>

namespace smb_roco {

RocoCommand::RocoCommand() : roco::CommandInterface() {
  romo_std::fillContainer<smb_description::ConcreteSmbDescription>(actuatorCommands_.wheelCommands_);
}

RocoCommand::~RocoCommand() {}

bool RocoCommand::limitCommand() {
  // wheel actuator commands could be limited in here
  return true;
}

std::ostream& operator<<(std::ostream& out, const RocoCommand& command) {
  for (const auto& actuatorKey : smb_description::SmbDescription::getKeys<smb_description::SmbTopology::SmbActuatorEnum>()) {
    const auto& actuatorEnum = actuatorKey.getEnum();
    const int actuatorIndex = actuatorKey.getId();
    out << "Actuator name: " << actuatorKey.getName() << std::endl;
    auto& smbActuatorCommand = command.actuatorCommands_.wheelCommands_[actuatorEnum];
    out << smbActuatorCommand;
  }
  return out;
}

void RocoCommand::setBaseTwistCommand(const any_measurements::Twist& baseTwistCommand) {
  double leftWheelSpeedCommand;
  double rightWheelSpeedCommand;
  skidSteerConversion(baseTwistCommand, leftWheelSpeedCommand, rightWheelSpeedCommand);

  for (const auto& actuatorKey : smb_description::SmbDescription::getKeys<smb_description::SmbTopology::SmbActuatorEnum>()) {
    const auto& actuatorEnum = actuatorKey.getEnum();
    auto& smbActuatorCommand = actuatorCommands_.wheelCommands_[actuatorEnum];

    smbActuatorCommand.setStamp(baseTwistCommand.time_);
    smbActuatorCommand.setMode(smb_common::SmbMode::MODE_WHEEL_VELOCITY);
    double command;
    if (actuatorEnum == smb_description::SmbTopology::SmbActuatorEnum::LF_WHEEL ||
        actuatorEnum == smb_description::SmbTopology::SmbActuatorEnum::LH_WHEEL)
      command = leftWheelSpeedCommand;
    else if (actuatorEnum == smb_description::SmbTopology::SmbActuatorEnum::RF_WHEEL ||
             actuatorEnum == smb_description::SmbTopology::SmbActuatorEnum::RH_WHEEL)
      command = rightWheelSpeedCommand;
    else
      continue;

    smbActuatorCommand.setWheelVelocity(command);
  }
}

void RocoCommand::freeze() {
  for (const auto& actuatorKey : smb_description::SmbDescription::getKeys<smb_description::SmbTopology::SmbActuatorEnum>()) {
    const auto& actuatorEnum = actuatorKey.getEnum();
    auto& smbActuatorCommand = actuatorCommands_.wheelCommands_[actuatorEnum];
    smbActuatorCommand.setMode(smb_common::SmbMode::FREEZE);
    smbActuatorCommand.setWheelVelocity(0);
  }
  lastForwardVelocity_ = 0;
  lastRotationalVelocity_ = 0;
}

void RocoCommand::skidSteerConversion(const any_measurements::Twist& twist, double& leftWheelVel, double& rightWheelVel) {
  double forwardVelocity = twist.twist_.getTranslationalVelocity().x();
  double rotationalVelocity = twist.twist_.getRotationalVelocity().z();

  applyLimits(forwardVelocity, rotationalVelocity);

  leftWheelVel = (forwardVelocity - wheelOffset_ * rotationalVelocity) / wheelRadius_;
  rightWheelVel = (forwardVelocity + wheelOffset_ * rotationalVelocity) / wheelRadius_;
}

void RocoCommand::applyLimits(double& forwardVelocity, double& rotationalVelocity) {
  double currentLinearAcceleration = (forwardVelocity - lastForwardVelocity_) / dt_;
  double currentRotationAcceleration = (rotationalVelocity - lastRotationalVelocity_) / dt_;

  if (currentLinearAcceleration > maxLinearAcceleration_) {
    MELO_INFO_THROTTLE_STREAM(1, "Linear acceleration limits exceeded. Limiting acceleration. currentLinearAcceleration: "
                                     << currentLinearAcceleration << " maxLinearAcceleration_: " << maxLinearAcceleration_);
    currentLinearAcceleration = maxLinearAcceleration_;
  } else if (currentLinearAcceleration < -maxLinearAcceleration_) {
    MELO_INFO_THROTTLE_STREAM(1, "Linear acceleration limits exceeded. Limiting acceleration. currentLinearAcceleration: "
                                     << currentLinearAcceleration << " minLinearAcceleration_: " << -maxLinearAcceleration_);
    currentLinearAcceleration = -maxLinearAcceleration_;
  }
  if (currentRotationAcceleration > maxRotationalAcceleration_) {
    MELO_INFO_THROTTLE_STREAM(1, "Rotational acceleration limits exceeded. Limiting acceleration. currentRotationAcceleration: "
                                     << currentRotationAcceleration << " maxRotationalAcceleration_: " << maxRotationalAcceleration_);
    currentRotationAcceleration = maxRotationalAcceleration_;
  } else if (currentRotationAcceleration < -maxRotationalAcceleration_) {
    MELO_INFO_THROTTLE_STREAM(1, "Rotational acceleration limits exceeded. Limiting acceleration. currentRotationAcceleration: "
                                     << currentRotationAcceleration << " minRotationalAcceleration_: " << -maxRotationalAcceleration_);
    currentRotationAcceleration = -maxRotationalAcceleration_;
  }

  forwardVelocity = lastForwardVelocity_ + currentLinearAcceleration * dt_;
  rotationalVelocity = lastRotationalVelocity_ + currentRotationAcceleration * dt_;

  if (forwardVelocity > maxLinearVelocity_) {
    MELO_INFO_THROTTLE_STREAM(1, "Linear velocity limits exceeded. Limiting velocity. forwardVelocity: "
                                     << forwardVelocity << " maxLinearVelocity_: " << maxLinearVelocity_);
    forwardVelocity = maxLinearVelocity_;
  } else if (forwardVelocity < -maxLinearVelocity_) {
    MELO_INFO_THROTTLE_STREAM(1, "Linear velocity limits exceeded. Limiting velocity. forwardVelocity: "
                                     << forwardVelocity << " minLinearVelocity_: " << -maxLinearVelocity_);
    forwardVelocity = -maxLinearVelocity_;
  }
  if (rotationalVelocity > maxRotationalVelocity_) {
    MELO_INFO_THROTTLE_STREAM(1, "Rotational velocity limits exceeded. Limiting velocity. rotationalVelocity: "
                                     << rotationalVelocity << " maxRotationalVelocity_: " << maxRotationalVelocity_);
    rotationalVelocity = maxRotationalVelocity_;
  } else if (rotationalVelocity < -maxRotationalVelocity_) {
    MELO_INFO_THROTTLE_STREAM(1, "Rotational velocity limits exceeded. Limiting velocity. rotationalVelocity: "
                                     << rotationalVelocity << " minRotationalVelocity_: " << -maxRotationalVelocity_);
    rotationalVelocity = -maxRotationalVelocity_;
  }

  lastForwardVelocity_ = forwardVelocity;
  lastRotationalVelocity_ = rotationalVelocity;
}

const RocoCommand::SmbCommandsShm& RocoCommand::getActuatorCommands() { return actuatorCommands_; }

void RocoCommand::setMaxLinearVelocity(double maxLinearVelocity) { RocoCommand::maxLinearVelocity_ = maxLinearVelocity; }

void RocoCommand::setMaxRotationalVelocity(double maxRotationalVelocity) { RocoCommand::maxRotationalVelocity_ = maxRotationalVelocity; }

void RocoCommand::setMaxLinearAcceleration(double maxLinearAcceleration) { RocoCommand::maxLinearAcceleration_ = maxLinearAcceleration; }

void RocoCommand::setMaxRotationalAcceleration(double maxRotationalAcceleration) {
  RocoCommand::maxRotationalAcceleration_ = maxRotationalAcceleration;
}

void RocoCommand::setDt(double dt) { dt_ = dt; }

} /* namespace smb_roco */