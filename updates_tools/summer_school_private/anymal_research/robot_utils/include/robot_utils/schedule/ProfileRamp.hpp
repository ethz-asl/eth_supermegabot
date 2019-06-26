/*!
* @file     ProfileRamp.hpp
* @author   Christian Gehring
* @date     April, 2015
* @version  1.0
* @ingroup  robot_utils
* @brief
*/

#pragma once

#include "robot_utils/schedule/Profile.hpp"

namespace robot_utils {

/*! Ramp profile
 *        /---
 *       /
 *      /
 *  ---/
 */
template<typename Value_>
class ProfileRamp: public Profile<Value_>
{
 public:
  typedef Value_ Value;
  typedef Profile<Value_> Base;

  ProfileRamp(const Value& startValue, const Value& endValue, double duration) :
    Base(duration),
    startValue_(startValue),
    endValue_(endValue)
  {
  }

  virtual ~ProfileRamp() {

  }

  virtual Value getValue(double time) {
    if (time < this->getStartTime()) {
      return startValue_;
    }
    else if(time > this->getEndTime()) {
      return endValue_;
    }
    return linearlyInterpolate(startValue_, endValue_, this->getStartTime(), this->getEndTime(), time);
  }

  virtual Value getValueFirstDerivative(double time) {
    if ( (time <= this->getStartTime()) || (time >= this->getEndTime()) ) {
      return 0.0*endValue_;
    }
    return (endValue_-startValue_)/(this->getEndTime()-this->getStartTime());
  }

 protected:
  inline Value linearlyInterpolate(const Value& v1, const Value& v2, double t1, double t2, double t){
    if (v1 == v2) {
      return v2;
    }
    return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
  }

 protected:
  Value startValue_;
  Value endValue_;

};

} /* namespace robot_utils */
