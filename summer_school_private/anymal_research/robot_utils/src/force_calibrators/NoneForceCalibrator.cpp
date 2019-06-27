/*
 * NoneForceCalibrator.cpp
 *
 *  Created on: Apr 6, 2016
 *      Author: Christian Gehring
 */

#include "robot_utils/force_calibrators/NoneForceCalibrator.hpp"
#include <message_logger/message_logger.hpp>

#include <stdio.h>
#include <stdlib.h>


namespace robot_utils {

NoneForceCalibrator::NoneForceCalibrator(const std::string& name) :
      ForceCalibratorBase(name)
{

}

bool NoneForceCalibrator::store(const std::string& filename) {
  return true;
}

bool NoneForceCalibrator::load(const std::string& filename) {
  return true;
}


bool NoneForceCalibrator::isCalibrating() {
  return false;
}

bool NoneForceCalibrator::startCalibration(bool waitFor) {
  return true;
}

bool NoneForceCalibrator::wait() {
  return true;
}

bool NoneForceCalibrator::initialize(double dt) {
  return true;
}

bool NoneForceCalibrator::advance(double dt) {
  return true;
}


bool NoneForceCalibrator::command(ForceCalibratorCommand& command) {
  return true;
}

bool NoneForceCalibrator::getCalibratedForce(kindr::Force3D& calibratedForce, const kindr::Force3D& uncalibratedForce) {
  calibratedForce = uncalibratedForce;
  return true;
}

bool NoneForceCalibrator::getCalibratedTorque(kindr::Torque3D& calibratedTorque, const kindr::Torque3D& uncalibratedTorque) {
  calibratedTorque = uncalibratedTorque;
  return true;
}

void NoneForceCalibrator::getStatistics(ForceCalibratorStats& statistics) {
  statistics.numSamples_ = 0u;
  statistics.numGoodSamples_ = 0u;
}

} /* namespace state_estimator */
