/*
 * ForceCalibratorBase.hpp
 *
 *  Created on: Jan 31, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <kindr/Core>
#include <boost/thread.hpp>
#include <string>

#include <robot_utils/force_calibrators/ForceCalibratorCommand.hpp>
#include <robot_utils/force_calibrators/ForceCalibratorStats.hpp>


namespace robot_utils {

class ForceCalibratorBase {
 public:
  ForceCalibratorBase(const std::string& name);
  virtual ~ForceCalibratorBase() = default;

  virtual bool store(const std::string& filename) = 0;
  virtual bool load(const std::string& filename) = 0;

  virtual bool isCalibrating() = 0;
  virtual bool startCalibration(bool wait = false) = 0;
  virtual bool wait() = 0;

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
  virtual bool command(ForceCalibratorCommand& command) = 0;

  virtual bool getCalibratedForce(kindr::Force3D& calibratedForce, const kindr::Force3D& uncalibratedForce) = 0;
  virtual bool getCalibratedTorque(kindr::Torque3D& calibratedTorque, const kindr::Torque3D& uncalibratedTorque) = 0;
  virtual bool getCalibratedWrench(kindr::WrenchD& calibratedWrench, const kindr::WrenchD& uncalibratedWrench) {
    bool success = getCalibratedForce(calibratedWrench.getForce(), uncalibratedWrench.getForce());
    success &= getCalibratedTorque(calibratedWrench.getTorque(), uncalibratedWrench.getTorque());
    return success;
  }

  virtual void getStatistics(ForceCalibratorStats& statistics) = 0;

  const std::string& getName() const {
    return name_;
  }

  void setWrench(const kindr::WrenchD& wrench) {
    boost::unique_lock<boost::shared_mutex> lock{mutexWrench_};
    wrench_ = wrench;
  }

  void setForce(const kindr::Force3D& force) {
    boost::unique_lock<boost::shared_mutex> lock{mutexWrench_};
    wrench_.setForce(force);
  }

  void setTorque(const kindr::Torque3D& torque) {
    boost::unique_lock<boost::shared_mutex> lock{mutexWrench_};
    wrench_.setTorque(torque);
  }

 protected:
  std::string name_;
  kindr::WrenchD wrench_;
  boost::shared_mutex mutexWrench_;
};

} /* namespace robot_utils */
