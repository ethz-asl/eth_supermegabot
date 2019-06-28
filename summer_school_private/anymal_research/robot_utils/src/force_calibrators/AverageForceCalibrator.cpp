/*
 * AverageForceCalibrator.cpp
 *
 *  Created on: Apr 6, 2016
 *      Author: Christian Gehring
 */

#include "robot_utils/force_calibrators/AverageForceCalibrator.hpp"
#include <message_logger/message_logger.hpp>

#include <stdio.h>
#include <stdlib.h>


namespace robot_utils {

AverageForceCalibrator::AverageForceCalibrator(const std::string& name, const Config& config) :
       ForceCalibratorBase(name),
      isCalibrating_(false),
      cvCalibrating_(),
      isSampling_(false),
      numGoodSamples_(0u),
      numSamples_(0u),
      wrenchOffset_(),
      config_(config)
{

}

bool AverageForceCalibrator::startCalibration(bool waitFor)
{
  if (config_.numSamples_ == 0u) {
    return false;
  }
  if (waitFor) {
    boost::unique_lock<boost::shared_mutex> lockConfig{mutexConfig_};
    config_.calibrateAfterSampling_ = true;
  }
  initSampling(false);
  isCalibrating_ = true;
  if (waitFor) {
    return wait();
  }
  MELO_INFO_STREAM("Start calibration of sensor " << name_);
  return true;
}


bool AverageForceCalibrator::continueSampling() {
  if (!isCalibrating_) {
    return false;
  }
  return initSampling(true);
}

bool AverageForceCalibrator::initSampling(bool continueSampling) {
  if (isSampling_ == false) {
    isSampling_ = true;
    numGoodSamples_ = 0u;
    numSamples_ = 0u;
    if (!continueSampling) {
      cmaFilter_.reset();
      // Update statistics
      {
        boost::unique_lock<boost::shared_mutex> lock{mutexStatistics_};
        statistics_.numSamples_ = 0u;
        statistics_.numGoodSamples_ = 0u;
      }
    }

  }
  return true;
}

bool AverageForceCalibrator::wait() {
  if (isCalibrating_ == false) {
    return true;
  }
  boost::unique_lock<boost::shared_mutex> lock{mutexCalibrating_};
  cvCalibrating_.wait(lock,[this](){return (isCalibrating_ == false);});

  boost::shared_lock<boost::shared_mutex> lockConfig{mutexConfig_};
  if (numGoodSamples_ < config_.minNumGoodSamples_) {
    return false;
  }
  return true;
}

bool AverageForceCalibrator::initialize(double dt) {
  return true;
}

bool AverageForceCalibrator::advance(double dt) {
  if (!isCalibrating_) {
    // exit
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////
  /// Sample
  ////////////////////////////////////////////////////////////////////////////
  if (isSampling_) {
    if ((!config_.enableOutlierDetectorForce_ || !isForceAnOutlier(wrench_.getForce(), wrenchOffset_.getForce()))
        && (!config_.enableOutlierDetectorTorque_ || !isTorqueAnOutlier(wrench_.getTorque(), wrenchOffset_.getTorque()))
        && (!config_.enableForceClamp_ || !isZForceOutsideClamp(wrench_.getForce().z()))
        ) {
      // This sample is good
      boost::shared_lock<boost::shared_mutex> lockWrench{mutexWrench_};
      boost::unique_lock<boost::shared_mutex> lockNewWrench{mutexCmaFilter_};
      cmaFilter_.update(wrench_);
      // We sampled successfully
      numGoodSamples_++;
//      std::cout << numGoodSamples_<< " Update good sample: " << wrench_  << std::endl;
    }

    numSamples_++;

    // Update statistics
    {
      boost::unique_lock<boost::shared_mutex> lock{mutexStatistics_};
      statistics_.numSamples_ = numSamples_;
      statistics_.numGoodSamples_ = numGoodSamples_;
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  /// Check if calibration has finished
  ////////////////////////////////////////////////////////////////////////////
  boost::shared_lock<boost::shared_mutex> lockConfig{mutexConfig_};
   if (numSamples_ >= config_.numSamples_) {
     isSampling_ = false;
     if (config_.calibrateAfterSampling_) {
       // Calibrate only if enough good samples have been collected.
       if (numGoodSamples_ >= config_.minNumGoodSamples_) {
         calibrateWrenchOffset();
       }
       // Finish calibration.
       notifyEndOfCalibration();
     }
   }
  return true;
}



void AverageForceCalibrator::calibrateWrenchOffset() {

  // Filter
  {
    boost::unique_lock<boost::shared_mutex> lock{mutexWrenchOffset_};
    boost::shared_lock<boost::shared_mutex> lockFilter{mutexCmaFilter_};

    wrenchOffset_.setForce(config_.alphaFilterForce_*cmaFilter_.getValue().getForce() + (1.0-config_.alphaFilterForce_)*wrenchOffset_.getForce());
    wrenchOffset_.setTorque(config_.alphaFilterForce_*cmaFilter_.getValue().getTorque() + (1.0-config_.alphaFilterTorque_)*wrenchOffset_.getTorque());
//    MELO_INFO_STREAM(name_ << ": cma: " << cmaFilter_.getValue() << " wrenchOffset: " << wrenchOffset_);
  }
}

bool AverageForceCalibrator::command(ForceCalibratorCommand& command) {



  if (command.cmdStart_) {
    command.cmdStart_ = false;

    // Start calibration only if it has not started yet.
    if (!isCalibrating_) {
      {
        boost::unique_lock<boost::shared_mutex> lock{mutexConfig_};
        config_.numSamples_ = command.numSamples_;
        config_.minNumGoodSamples_ = command.numGoodSamples_;
        config_.calibrateAfterSampling_ = command.cmdCalibrate_;
        config_.enableOutlierDetectorForce_ = command.enableOutlierDetector_;
        config_.enableOutlierDetectorTorque_ = command.enableOutlierDetector_;
      }
      startCalibration(false);
      return true;
    }
  }
  if (command.cmdContinue_) {
    command.cmdContinue_ = false;

    // Continue only if the calibration has started and it is not sampling already.
    if (isCalibrating_ && !isSampling_) {
      if (!isSampling_) {
        {
          boost::unique_lock<boost::shared_mutex> lock{mutexConfig_};
          config_.numSamples_ = command.numSamples_;
          config_.minNumGoodSamples_ = command.numGoodSamples_;
          config_.calibrateAfterSampling_ = command.cmdCalibrate_;
          config_.enableOutlierDetectorForce_ = command.enableOutlierDetector_;
          config_.enableOutlierDetectorTorque_ = command.enableOutlierDetector_;
        }
        continueSampling();
      }
    }
  }
  return true;
}

void AverageForceCalibrator::notifyEndOfCalibration() {
  isCalibrating_ = false;
  isSampling_ = false;
  cvCalibrating_.notify_all();
}

bool AverageForceCalibrator::getCalibratedForce(kindr::Force3D& calibratedForce, const kindr::Force3D& uncalibratedForce) {
  calibratedForce = uncalibratedForce-wrenchOffset_.getForce();
  return true;
}

bool AverageForceCalibrator::getCalibratedTorque(kindr::Torque3D& calibratedTorque, const kindr::Torque3D& uncalibratedTorque) {
  calibratedTorque = uncalibratedTorque-wrenchOffset_.getTorque();
  return true;
}

bool AverageForceCalibrator::isCalibrating() {
  return isCalibrating_;
}

void AverageForceCalibrator::stopCalibration() {
  boost::shared_lock<boost::shared_mutex> lock{mutexConfig_};
  // Calibrate only if enough good samples have been collected.
  if (numGoodSamples_ >= config_.minNumGoodSamples_) {
//    std::cout << "stopCalibration: calibrate wrench offset\n";
    calibrateWrenchOffset();
  }
  // Finish calibration.
  notifyEndOfCalibration();
}

bool AverageForceCalibrator::isForceAnOutlier(const kindr::Force3D& sample, const kindr::Force3D& mean) {
  boost::shared_lock<boost::shared_mutex> lockConfig{mutexConfig_};
  const auto error = sample-mean;
  const double mahalanobisDistance = std::sqrt(error.x()*error.x()/config_.outlierDetectionVariance_.getForce().x() + error.y()*error.y()/config_.outlierDetectionVariance_.getForce().y() + error.z()*error.z()/config_.outlierDetectionVariance_.getForce().z());
  const bool isOutlier = (mahalanobisDistance > config_.outlierDetectionMaxMahalanobisDistanceForce_);
  if (isOutlier) {
    MELO_WARN("Calibrator %s: Detected outlier in force! (Mahalanobis distance: %lf > %lf)", name_.c_str(), (double)mahalanobisDistance, config_.outlierDetectionMaxMahalanobisDistanceForce_);
  }
  return  isOutlier;
}

bool AverageForceCalibrator::isTorqueAnOutlier(const kindr::Torque3D& sample, const kindr::Torque3D& mean) {
  boost::shared_lock<boost::shared_mutex> lockConfig{mutexConfig_};
  const auto error = sample-mean;
  const double mahalanobisDistance = std::sqrt(error.x()*error.x()/config_.outlierDetectionVariance_.getTorque().x() + error.y()*error.y()/config_.outlierDetectionVariance_.getTorque().y() + error.z()*error.z()/config_.outlierDetectionVariance_.getTorque().z());
  const bool isOutlier = (mahalanobisDistance > config_.outlierDetectionMaxMahalanobisDistanceTorque_);
  if (isOutlier) {
    MELO_WARN("Calibrator %s: Detected outlier in torque! (Mahalanobis distance: %lf > %lf)", name_.c_str(), (double)mahalanobisDistance, config_.outlierDetectionMaxMahalanobisDistanceForce_);
  }
  return  isOutlier;
}

bool AverageForceCalibrator::isZForceOutsideClamp(double zForce) {
  boost::shared_lock<boost::shared_mutex> lockConfig{mutexConfig_};
  if (zForce > config_.lowerForceThreshold_ && wrench_.getForce().z() < config_.upperForceThreshold_) {
	return false;
  } else {
	MELO_WARN("Calibrator %s: Detected outlier in force in z direction! (Force in z direction: %lf < %lf or > %lf)", name_.c_str(), zForce, config_.lowerForceThreshold_, config_.upperForceThreshold_);
    return true;
  }
}

bool AverageForceCalibrator::store(const std::string& filename) {
  FILE * fp = fopen(filename.c_str(), "w+");
  if (!fp) {
    MELO_ERROR_STREAM("State estimator: Could not store force calibration " << name_ << " to file: " <<  filename);
    return false;
  }
  boost::shared_lock<boost::shared_mutex> lock{mutexWrenchOffset_};
  fprintf(fp, "%lf %lf %lf %lf %lf %lf",
          wrenchOffset_.getForce().x(),
          wrenchOffset_.getForce().y(),
          wrenchOffset_.getForce().z(),
          wrenchOffset_.getTorque().x(),
          wrenchOffset_.getTorque().y(),
          wrenchOffset_.getTorque().z());
  fclose(fp);

  MELO_INFO("AverageForceCalibrator: Stored wrench offset: %lf %lf %lf %lf %lf %lf for sensor %s.\n",
         wrenchOffset_.getForce().x(),
         wrenchOffset_.getForce().y(),
         wrenchOffset_.getForce().z(),
         wrenchOffset_.getTorque().x(),
         wrenchOffset_.getTorque().y(),
         wrenchOffset_.getTorque().z(),
         name_.c_str());

  MELO_INFO_STREAM("AverageForceCalibrator: Stored force calibration " << name_ << " to file: " <<  filename);
  return true;
}

bool AverageForceCalibrator::load(const std::string& filename) {
  FILE * fp = fopen(filename.c_str(), "r");
  if (!fp) {
    return false;
  }
  boost::unique_lock<boost::shared_mutex> lock{mutexWrenchOffset_};
  int res = fscanf(fp, "%lf %lf %lf %lf %lf %lf",
         &wrenchOffset_.getForce().x(),
         &wrenchOffset_.getForce().y(),
         &wrenchOffset_.getForce().z(),
         &wrenchOffset_.getTorque().x(),
         &wrenchOffset_.getTorque().y(),
         &wrenchOffset_.getTorque().z());
  fclose(fp);
  MELO_INFO("AverageForceCalibrator: Loaded force offset: %lf %lf %lf %lf %lf %lf for sensor %s.\n",
            wrenchOffset_.getForce().x(),
           wrenchOffset_.getForce().y(),
           wrenchOffset_.getForce().z(),
           wrenchOffset_.getTorque().x(),
           wrenchOffset_.getTorque().y(),
           wrenchOffset_.getTorque().z(),
         name_.c_str());
  return true;
}


} /* namespace state_estimator */
