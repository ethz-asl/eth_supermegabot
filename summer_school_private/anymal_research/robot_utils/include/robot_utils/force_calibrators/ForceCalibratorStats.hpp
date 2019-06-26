/*
 * ForceCalibratorCommand.hpp
 *
 *  Created on: Feb 10, 2017
 *      Author: Christian Gehring
 */

#pragma once

namespace robot_utils {

//! Statistics of the force calibrator.
struct ForceCalibratorStats {

  //! Number of samples that should be sampled for the calibration.
  unsigned int numSamples_ = 0u;

  //! Required minimal number of samples for the calibration.
  unsigned int numGoodSamples_ = 0u;


};

} /* namespace robot_utils */
