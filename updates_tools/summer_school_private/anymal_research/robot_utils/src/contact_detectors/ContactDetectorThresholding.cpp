/*
 * ContactDetectorThresholding.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Christian Gehring
 */

#include "robot_utils/contact_detectors/ContactDetectorThresholding.hpp"
#include <message_logger/message_logger.hpp>

namespace robot_utils {

ContactDetectorThresholding::ContactDetectorThresholding(const std::string& name, const Config& config) :
    ContactDetectorBase(name),
    config_(config)
{

}

ContactDetectorThresholding::~ContactDetectorThresholding() {

}

bool ContactDetectorThresholding::initialize(double dt) {
  return detectContact();
}

bool ContactDetectorThresholding::advance(double dt) {
  return detectContact();
}

bool ContactDetectorThresholding::detectContact() {
  boost::shared_lock<boost::shared_mutex> lock{mutexConfig_};
  if (config_.isThresholdingForce_ && !config_.isThresholdingTorque_) {
    state_ = getStateFromForce();
    return true;
  }
  MELO_ERROR("[ContactDetectorThresholding] The configuration is not yet supported!");
  return false;
}

ContactDetectorBase::ContactState ContactDetectorThresholding::getStateFromForce() {
  boost::shared_lock<boost::shared_mutex> lock{mutexConfig_};
  switch(config_.forceThresholdingMethod_) {
    case ThresholdingMethod::NORM:
      if (wrench_.getForce().norm() > config_.thresholdForceNorm_) {
        return ContactState::CLOSED;
      }
      else {
        return ContactState::OPEN;
      }
    break;
    case ThresholdingMethod::XYONLY:
      if (std::abs(wrench_.getForce().x()*wrench_.getForce().x() + wrench_.getForce().y()*wrench_.getForce().y()) > config_.thresholdForceNorm_) {
        return ContactState::CLOSED;
      }
      else {
        return ContactState::OPEN;
      }
      break;
    case ThresholdingMethod::ZONLY:
      if (std::abs(wrench_.getForce().z()) > config_.thresholdForce_.z()) {
        return ContactState::CLOSED;
      }
      else {
        return ContactState::OPEN;
      }
      break;
    case ThresholdingMethod::XONLY:
      if (std::abs(wrench_.getForce().x()) > config_.thresholdForce_.x()) {
        return ContactState::CLOSED;
      }
      else {
        return ContactState::OPEN;
      }
      break;
    case ThresholdingMethod::ZCLAMPED:
      if (wrench_.getForce().z() > config_.thresholdForce_.z() &&
          wrench_.getForce().z() < config_.upperThresholdForce_.z()) {
        return ContactState::CLOSED;
      }
      else {
        return ContactState::OPEN;
      }
      break;
    case ThresholdingMethod::ALL:
      if ( (std::abs(wrench_.getForce().x()) > config_.thresholdForce_.x()) ||
           (std::abs(wrench_.getForce().y()) > config_.thresholdForce_.y()) ||
           (std::abs(wrench_.getForce().z()) > config_.thresholdForce_.z())
      ) {
        return ContactState::CLOSED;
      }
      else {
        return ContactState::OPEN;
      }
      break;
    case ThresholdingMethod::ALLCLAMPED:
      if ( (std::abs(wrench_.getForce().x()) > config_.thresholdForce_.x() &&
            std::abs(wrench_.getForce().x()) < config_.upperThresholdForce_.x()) ||
           (std::abs(wrench_.getForce().y()) > config_.thresholdForce_.y() &&
            std::abs(wrench_.getForce().y()) < config_.upperThresholdForce_.y()) ||
           (std::abs(wrench_.getForce().z()) > config_.thresholdForce_.z() &&
            std::abs(wrench_.getForce().z()) < config_.upperThresholdForce_.z())
      ) {
        return ContactState::CLOSED;
      }
      else {
        return ContactState::OPEN;
      }
      break;
  }
  return ContactState::OPEN;
}

} /* namespace state_estimator */
