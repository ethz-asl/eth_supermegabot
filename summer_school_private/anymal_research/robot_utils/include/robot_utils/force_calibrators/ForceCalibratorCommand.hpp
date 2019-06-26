/*
 * ForceCalibratorCommand.hpp
 *
 *  Created on: Feb 10, 2017
 *      Author: Christian Gehring
 */

#pragma once

namespace robot_utils {

//! Command to control the force calibrator.
struct ForceCalibratorCommand {
  //! Command to start the calibration.
  bool cmdStart_ = false;

  //! Command to continue sampling. Note that the calibration needs to be started first.
  bool cmdContinue_ = false;

  //! Command to calibrate after the sampling process.
  bool cmdCalibrate_ = true;

  //! If true, the outlier detector is active
  bool enableOutlierDetector_ = true;

  //! Number of samples that should be sampled for the calibration.
  unsigned int numSamples_ = 1u;

  //! Required minimal number of samples for the calibration.
  unsigned int numGoodSamples_ = 1u;


};

} /* namespace robot_utils */
