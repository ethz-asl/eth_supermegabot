/*!
* @file     ProfileLogChirpUpSweep.hpp
* @author   Christian Gehring
* @date     April, 2015
* @version  1.0
* @ingroup  robot_utils
* @brief
*/

#pragma once

#include "robot_utils/schedule/Profile.hpp"
#include "robot_utils/function_generators/FctLogChirp.hpp"

namespace robot_utils {

template<typename Value_>
class ProfileLogChirpUpSweep: public Profile<double>
{
 public:
  typedef double Value;
  typedef Profile<Value> Base;
  ProfileLogChirpUpSweep(const Value& offsetValue,
                  const Value& maxValue,
                  double minFrequencyHz,
                  double maxFrequencyHz,
                  double duration) :
    Base(duration),
    logChirp_()
  {
    logChirp_.setParamAmplitude(maxValue);
    logChirp_.setParamMinFrequencyHz(minFrequencyHz);
    logChirp_.setParamMaxFrequencyHz(maxFrequencyHz);
    logChirp_.setParamTimeInteval(duration);
    logChirp_.setParamOffset(offsetValue);
  }

  virtual ~ProfileLogChirpUpSweep() {

  }
  virtual Value getValue(double time) {
    if (time < this->getStartTime()) {
      return logChirp_.getParamOffset();
    }
    else if (time > this->getEndTime()) {
      return logChirp_.getParamOffset();
    }
    return logChirp_.getUpSweepValue(time-this->getStartTime());
  }

  virtual Value getValueFirstDerivative(double time) {
    return logChirp_.getUpSweepFirstDerivativeValue(time-this->getStartTime());
  }

 protected:
  FctLogChirp logChirp_;


};

} /* namespace robot_utils */
