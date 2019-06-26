/*
 * ContactDetectorThresholding.hpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include "robot_utils/contact_detectors/ContactDetectorBase.hpp"
#include <boost/thread.hpp>
#include <string>

namespace robot_utils {


class ContactDetectorThresholding : public ContactDetectorBase {
 public:
  enum class ThresholdingMethod : unsigned int {
    NORM = 0,
    ZONLY,
    ZCLAMPED,
    ALL,
    ALLCLAMPED,
    XONLY,
    XYONLY
  };
  class Config {
   public:
    Config() {};
    virtual ~Config() {}
    bool isThresholdingForce_ = true;
    bool isThresholdingTorque_ = true;
    ThresholdingMethod forceThresholdingMethod_ = ThresholdingMethod::ALL;
    ThresholdingMethod torqueThresholdingMethod_ = ThresholdingMethod::ALL;
    kindr::Force3D thresholdForce_;
    kindr::Force3D upperThresholdForce_;
    kindr::Torque3D thresholdTorque_;
    kindr::Torque3D upperThresholdTorque_;
    double thresholdForceNorm_ = 0.0;
    double thresholdTorqueNorm_ = 0.0;

    friend std::ostream & operator << (std::ostream & out, const Config& config) {
      out << "isThresholdingForce: " << (config.isThresholdingForce_ ? "yes" : "no") << std::endl;
      out << "isThresholdingTorque: " << (config.isThresholdingTorque_ ? "yes" : "no") << std::endl;
      out << "forceThresholdingMethod: " << static_cast<unsigned int>(config.forceThresholdingMethod_) << std::endl;
      out << "torqueThresholdingMethod: " << static_cast<unsigned int>(config.torqueThresholdingMethod_) << std::endl;
      out << "thresholdForceNorm: " << config.thresholdForceNorm_ << std::endl;
      out << "thresholdForce: " << config.thresholdForce_ << std::endl;
      out << "upperThresholdForce: " << config.upperThresholdForce_ << std::endl;
      out << "thresholdTorqueNorm: " << config.thresholdTorqueNorm_ << std::endl;
      out << "upperThresholdTorque: " << config.upperThresholdTorque_ << std::endl;
      out << "thresholdTorque: " << config.thresholdTorque_ << std::endl;
      return out;
    }
  };

 public:
  ContactDetectorThresholding(const std::string& name, const Config& config = Config());
  virtual ~ContactDetectorThresholding();
  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  void setConfig(const Config& config) {
    boost::unique_lock<boost::shared_mutex> lock{mutexConfig_};
    config_ = config;
  }
 protected:
  bool detectContact();
  ContactDetectorBase::ContactState getStateFromForce();

 protected:
  Config config_;
  boost::shared_mutex mutexConfig_;

};

} /* namespace state_estimator */
