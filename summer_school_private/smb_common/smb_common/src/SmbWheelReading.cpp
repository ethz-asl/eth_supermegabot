/*!
 * @file    SmbWheelReading.cpp
 * @author  Koen Kraemer
 * @date    Aug 2, 2018
 *
 */

// smb_common
#include "smb_common/SmbWheelReading.hpp"


namespace smb_common {


SmbWheelReading::SmbWheelReading() {}

SmbWheelReading::~SmbWheelReading() {}


const any_measurements::Time& SmbWheelReading::getStamp() const {
  return stamp_;
}

void SmbWheelReading::setStamp(const any_measurements::Time& stamp) {
  stamp_ = stamp;
}

const double& SmbWheelReading::getCurrent() const {
  return current_;
}

void SmbWheelReading::setCurrent(double current) {
  current_ = current;
}

const double& SmbWheelReading::getWheelVelocity() const {
  return wheelVelocity_;
}

void SmbWheelReading::setWheelVelocity(double wheelVelocity) {
  wheelVelocity_ = wheelVelocity;
}

const double& SmbWheelReading::getWheelTorque() const {
  return wheelTorque_;
}

void SmbWheelReading::setWheelTorque(double wheelTorque) {
  wheelTorque_ = wheelTorque;
}

std::ostream& operator<<(std::ostream& out, const SmbWheelReading& reading) {
  out << "Current: " << reading.current_ << std::endl;
  out << "Wheel velocity: " << reading.wheelVelocity_ << std::endl;
  out << "Wheel torque: " << reading.wheelTorque_ << std::endl;
  return out;
}


} // smb_common
