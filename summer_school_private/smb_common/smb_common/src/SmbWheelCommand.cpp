/*!
 * @file    SmbWheelCommand.cpp
 * @author  Koen Kraemer
 * @date    Aug 2, 2018
 *
 */

// c++
#include <cmath>
#include <iomanip>
#include <unordered_map>

// smb_common
#include "smb_common/SmbWheelCommand.hpp"


namespace smb_common {

static std::unordered_map<SmbModeType, std::string> modeNames = {
    { SmbMode::FREEZE, "No commands sent (estop)" },
    { SmbMode::MODE_WHEEL_VELOCITY, "Track wheel velocity" },
    { SmbMode::MODE_WHEEL_TORQUE, "Track wheel torque" },
    { SmbMode::WHEEL_DC_CMD, "Command motor duty cycle open loop" },
};


SmbWheelCommand::SmbWheelCommand()
: SmbWheelCommand(SmbMode::FREEZE) {}

SmbWheelCommand::SmbWheelCommand(SmbModeType mode)
: mode_(mode) {}

SmbWheelCommand::~SmbWheelCommand() {}

const any_measurements::Time& SmbWheelCommand::getStamp() const {
  return stamp_;
}

void SmbWheelCommand::setStamp(const any_measurements::Time& stamp) {
  stamp_ = stamp;
}

std::string SmbWheelCommand::getModeName(SmbModeType mode) {
  try {
    return modeNames.at(mode);
  }
  catch (...) {
    return std::string("Unknown mode: ") + std::to_string(mode);
  }
}

std::string SmbWheelCommand::getModeName() const {
  return getModeName(static_cast<SmbModeType>(mode_));
}

const SmbModeType& SmbWheelCommand::getMode() const {
  return mode_;
}

void SmbWheelCommand::setMode(const SmbModeType& mode) {
  mode_ = mode;
}

SmbMode SmbWheelCommand::getModeEnum() const {
  return static_cast<SmbMode>(mode_);
}

void SmbWheelCommand::setModeEnum(const SmbMode& modeEnum) {
  mode_ = static_cast<SmbModeType>(modeEnum);
}

const double& SmbWheelCommand::getCurrent() const {
  return current_;
}

void SmbWheelCommand::setCurrent(double current) {
  current_ = current;
}

const double& SmbWheelCommand::getWheelVelocity() const {
  return wheelVelocity_;
}

void SmbWheelCommand::setWheelVelocity(double wheelVelocity) {
  wheelVelocity_ = wheelVelocity;
}

const double& SmbWheelCommand::getWheelTorque() const {
  return wheelTorque_;
}

void SmbWheelCommand::setWheelTorque(double wheelTorque) {
  wheelTorque_ = wheelTorque;
}

// const double& SmbWheelCommand::getCurrentMax() const {
//   return currentMax_;
// }

// void SmbWheelCommand::setCurrentMax(double currentMax) {
//   currentMax_ = currentMax;
// }

// const double& SmbWheelCommand::getCurrentMin() const {
//   return currentMin_;
// }

// void SmbWheelCommand::setCurrentMin(double currentMin) {
//   currentMin_ = currentMin;
// }

// const double& SmbWheelCommand::getMotorPositionMin() const {
//   return motorPositionMin_;
// }

// void SmbWheelCommand::setMotorPositionMin(double motorPositionMin) {
//   motorPositionMin_ = motorPositionMin;
// }

// const double& SmbWheelCommand::getMotorPositionMax() const {
//   return motorPositionMax_;
// }

// void SmbWheelCommand::setMotorPositionMax(double motorPositionMax) {
//   motorPositionMax_ = motorPositionMax;
// }

// const double& SmbWheelCommand::getMotorVelocityMin() const {
//   return motorVelocityMin_;
// }

// void SmbWheelCommand::setMotorVelocityMin(double motorVelocityMin) {
//   motorVelocityMin_ = motorVelocityMin;
// }

// const double& SmbWheelCommand::getMotorVelocityMax() const {
//   return motorVelocityMax_;
// }

// void SmbWheelCommand::setMotorVelocityMax(double motorVelocityMax) {
//   motorVelocityMax_ = motorVelocityMax;
// }

// const double& SmbWheelCommand::getGearPositionMin() const {
//   return gearPositionMin_;
// }

// void SmbWheelCommand::setGearPositionMin(double gearPositionMin) {
//   gearPositionMin_ = gearPositionMin;
// }

// const double& SmbWheelCommand::getGearPositionMax() const {
//   return gearPositionMax_;
// }

// void SmbWheelCommand::setGearPositionMax(double gearPositionMax) {
//   gearPositionMax_ = gearPositionMax;
// }

// const double& SmbWheelCommand::getGearVelocityMin() const {
//   return gearVelocityMin_;
// }

// void SmbWheelCommand::setGearVelocityMin(double gearVelocityMin) {
//   gearVelocityMin_ = gearVelocityMin;
// }

// const double& SmbWheelCommand::getGearVelocityMax() const {
//   return gearVelocityMax_;
// }

// void SmbWheelCommand::setGearVelocityMax(double gearVelocityMax) {
//   gearVelocityMax_ = gearVelocityMax;
// }

// const double& SmbWheelCommand::getJointPositionMin() const {
//   return jointPositionMin_;
// }

// void SmbWheelCommand::setJointPositionMin(double jointPositionMin) {
//   jointPositionMin_ = jointPositionMin;
// }

// const double& SmbWheelCommand::getJointPositionMax() const {
//   return jointPositionMax_;
// }

// void SmbWheelCommand::setJointPositionMax(double jointPositionMax) {
//   jointPositionMax_ = jointPositionMax;
// }

// const double& SmbWheelCommand::getJointVelocityMin() const {
//   return jointVelocityMin_;
// }

// void SmbWheelCommand::setJointVelocityMin(double jointVelocityMin) {
//   jointVelocityMin_ = jointVelocityMin;
// }

// const double& SmbWheelCommand::getJointVelocityMax() const {
//   return jointVelocityMax_;
// }

// void SmbWheelCommand::setJointVelocityMax(double jointVelocityMax) {
//   jointVelocityMax_ = jointVelocityMax;
// }

// const double& SmbWheelCommand::getJointTorqueMin() const {
//   return jointTorqueMin_;
// }

// void SmbWheelCommand::setJointTorqueMin(double jointTorqueMin) {
//   jointTorqueMin_ = jointTorqueMin;
// }

// const double& SmbWheelCommand::getJointTorqueMax() const {
//   return jointTorqueMax_;
// }

// void SmbWheelCommand::setJointTorqueMax(double jointTorqueMax) {
//   jointTorqueMax_ = jointTorqueMax;
// }

void SmbWheelCommand::limit() {
  // TODO add saturation function
  // current_ = saturate(current_, currentMin_, currentMax_);
  // jointVelocity_ = saturate(jointVelocity_, jointVelocityMin_, jointVelocityMax_);
  // jointTorque_ = saturate(jointTorque_, jointTorqueMin_, jointTorqueMax_);
}

bool SmbWheelCommand::isFinite() const {
  return (
      std::isfinite(wheelVelocity_) &&
      std::isfinite(wheelTorque_));
}

bool SmbWheelCommand::isWithinLimits() const {
  return true;
  // (
  //     current_ >= currentMin_ &&
  //     current_ <= currentMax_ &&
  //     jointVelocity_ >= jointVelocityMin_ &&
  //     jointVelocity_ <= jointVelocityMax_ &&
  //     jointTorque_ >= jointTorqueMin_ &&
  //     jointTorque_ <= jointTorqueMax_);
}

bool SmbWheelCommand::isValid() const {
  return isFinite() && isWithinLimits();
}

std::ostream& operator<<(std::ostream& out, const SmbWheelCommand& command) {
  out << "mode: "  << command.getModeName() << std::endl;
  out << "current: "  << command.current_ << std::endl;
  out << "wheel velocity: "  << command.wheelVelocity_ << std::endl;
  out << "wheel torque: "  << command.wheelTorque_ << std::endl;
  return out;
}

} // smb_common


