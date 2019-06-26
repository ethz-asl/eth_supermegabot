/*!
* @file     ProfileStep.hpp
* @author   Christian Gehring
* @date     April, 2015
* @version  1.0
* @ingroup  robot_utils
* @brief
*/

#pragma once

#include "robot_utils/schedule/Profile.hpp"

namespace robot_utils {

template<typename Value_>
class ProfileStep: public Profile<Value_>
{
 public:
  typedef Value_ Value;
  typedef Profile<Value_> Base;
  ProfileStep(const Value& value, double duration) :
    Base(duration),
    value_(value)
  {

  }

  virtual ~ProfileStep() {

  }
  virtual Value getValue(double time) {
    return value_;
  }

  virtual Value getValueFirstDerivative(double time) {
    return 0.0*value_;
  }

 protected:
  Value value_;


};

} /* namespace robot_utils */
