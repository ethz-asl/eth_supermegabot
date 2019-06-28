/*
 * NoneForceCalibrator.hpp
 *
 *  Created on: Feb 21, 2018
 *      Author: Markus Staeuble
 */

#pragma once

#include <kindr/Core>
#include <boost/thread.hpp>
#include <string>

#include <robot_utils/force_calibrators/ForceCalibratorBase.hpp>
#include <robot_utils/force_calibrators/ForceCalibratorCommand.hpp>
#include <robot_utils/force_calibrators/ForceCalibratorStats.hpp>


namespace robot_utils {

/**
 * @brief      This calibrator forwards the uncalibrated wrench
 */
class NoneForceCalibrator : public ForceCalibratorBase {
 public:
  NoneForceCalibrator(const std::string& name);
  ~NoneForceCalibrator() override = default;

  bool store(const std::string& filename) override;
  bool load(const std::string& filename) override;

  bool isCalibrating() override;
  bool startCalibration(bool wait = false) override;
  bool wait() override;

  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool command(ForceCalibratorCommand& command) override;

  bool getCalibratedForce(kindr::Force3D& calibratedForce, const kindr::Force3D& uncalibratedForce) override;
  bool getCalibratedTorque(kindr::Torque3D& calibratedTorque, const kindr::Torque3D& uncalibratedTorque) override;

  void getStatistics(ForceCalibratorStats& statistics) override;
};

} /* namespace robot_utils */
