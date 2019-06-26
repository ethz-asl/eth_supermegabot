/*
 * AverageForceCalibrator.hpp
 *
 *  Created on: Apr 6, 2016
 *      Author: Christian Gehring
 */

#pragma once

#include "robot_utils/force_calibrators/ForceCalibratorBase.hpp"
#include "robot_utils/force_calibrators/ForceCalibratorStats.hpp"
#include "robot_utils/filters/CumulativeMovingAverageFilter.hpp"

#include <boost/thread.hpp>
#include <kindr/Core>

namespace robot_utils {

//! Calibrates the force offset using a Cumulative Moving Average filter.
class AverageForceCalibrator : public ForceCalibratorBase
{
 public:
	  class Config {
	   public:
	    Config() {};
	    virtual ~Config() = default;
	    bool calibrateAfterSampling_ = true;
      unsigned int numSamples_ = 1000u;
      unsigned int minNumGoodSamples_ = 500u;
	    double alphaFilterForce_ = 1.0;
	    double alphaFilterTorque_ = 1.0;
	    bool enableOutlierDetectorForce_ = true;
	    bool enableOutlierDetectorTorque_ = true;
      bool enableForceClamp_ = true;
      double lowerForceThreshold_ = -100.0;
      double upperForceThreshold_ = 100.0;
	    double outlierDetectionMaxMahalanobisDistanceForce_ = 1.0e18;
      double outlierDetectionMaxMahalanobisDistanceTorque_ = 1.0e18;
	    kindr::WrenchD outlierDetectionVariance_ = kindr::WrenchD( kindr::Force3D(1.0, 1.0, 1.0), kindr::Torque3D(1.0, 1.0, 1.0));

      friend std::ostream & operator << (std::ostream & out, const Config& config) {
        out << "numSamples: " << config.numSamples_ << std::endl;
        out << "minNumGoodSamples: " << config.minNumGoodSamples_ << std::endl;
        out << "alphaFilterForce: " << config.alphaFilterForce_ << std::endl;
        out << "alphaFilterTorque: " << config.alphaFilterTorque_ << std::endl;
        out << "enableOutlierDetectorForce: " << (config.enableOutlierDetectorForce_ ? "yes" : "no") << std::endl;
        out << "enableOutlierDetectorTorque: " << (config.enableOutlierDetectorForce_ ? "yes" : "no") << std::endl;
        out << "enableForceClamp: " << (config.enableForceClamp_ ? "yes" : "no") << std::endl;
        out << "lowerForceThreshold: " << config.lowerForceThreshold_ << std::endl;
        out << "upperForceThreshold: " << config.upperForceThreshold_ << std::endl;
        out << "outlierDetectionMaxMahalanobisDistanceForce: " << config.outlierDetectionMaxMahalanobisDistanceForce_ << std::endl;
        out << "outlierDetectionMaxMahalanobisDistanceTorque: " << config.outlierDetectionMaxMahalanobisDistanceTorque_ << std::endl;
        out << "outlierDetectionVariance: " << config.outlierDetectionVariance_ << std::endl;
        out << "calibrateAfterSampling: " << (config.calibrateAfterSampling_ ? "yes" : "no") << std::endl;
        return out;
      }
	  };
 public:
  AverageForceCalibrator(const std::string& name, const Config& config = Config());
  ~AverageForceCalibrator() override = default;

  bool getCalibratedForce(kindr::Force3D& calibratedForce, const kindr::Force3D& uncalibratedForce) override;
  bool getCalibratedTorque(kindr::Torque3D& calibratedTorque, const kindr::Torque3D& uncalibratedTorque) override;

  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool command(ForceCalibratorCommand& command) override;
  bool isCalibrating() override;
  bool startCalibration(bool wait = false) override;
  void stopCalibration();
  bool continueSampling();
  bool wait() override;

  bool store(const std::string& filename) override;
  bool load(const std::string& filename) override;

  void setConfig(const Config& config) {
    boost::unique_lock<boost::shared_mutex> lock{mutexConfig_};
    config_ = config;
  }

  void getConfig(Config& config) {
    boost::shared_lock<boost::shared_mutex> lock{mutexConfig_};
    config = config_;
  }
  void getWrenchOffset(kindr::WrenchD& wrenchOffset) {
    boost::shared_lock<boost::shared_mutex> lock{mutexWrenchOffset_};
    wrenchOffset = wrenchOffset_;
  }

  void getStatistics(ForceCalibratorStats& statistics) override {
    boost::shared_lock<boost::shared_mutex> lock{mutexStatistics_};
    statistics = statistics_;
  }

 protected:
  bool initSampling(bool continueSampling);
  void calibrateWrenchOffset();
  void notifyEndOfCalibration();
  bool isForceAnOutlier(const kindr::Force3D& sample, const kindr::Force3D& mean);
  bool isTorqueAnOutlier(const kindr::Torque3D& sample, const kindr::Torque3D& mean);
  bool isZForceOutsideClamp(double zForce);

 protected:
  //! Indicates whether the calibration process is ongoing.
  boost::atomic<bool> isCalibrating_;
  boost::condition_variable_any cvCalibrating_;
  boost::shared_mutex mutexCalibrating_;

  //! Indicates wether the sampling process within the calibration process is ongoing.
  boost::atomic<bool> isSampling_;

  //! Number of good samples collected during one sampling process.
  unsigned int numGoodSamples_;
  //! Number of samples collected during one sampling process.
  unsigned int numSamples_;

  kindr::WrenchD wrenchOffset_;
  boost::shared_mutex mutexWrenchOffset_;

  CumulativeMovingAverageFilter<kindr::WrenchD> cmaFilter_;
  boost::shared_mutex mutexCmaFilter_;

  Config config_;
  boost::shared_mutex mutexConfig_;

  ForceCalibratorStats statistics_;
  boost::shared_mutex mutexStatistics_;

};

} /* namespace state_estimator */
