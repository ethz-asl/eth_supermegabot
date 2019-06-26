/*!
* @file     Profile.hpp
* @author   Christian Gehring
* @date     April, 2015
* @version  1.0
* @ingroup  robot_utils
* @brief
*/

#pragma once

namespace robot_utils {

template<typename Value_>
class Profile
{
 public:
  typedef Value_ Value;

  Profile(double duration) :
    Profile(0.0, duration)
  {

  }
  Profile(double startTime, double duration) :
    startTime_(startTime),
    duration_(duration)
  {

  }
  virtual ~Profile() {

  }


  virtual Value getValue(double time) = 0;
  virtual Value getValueFirstDerivative(double time) = 0;
  virtual Value getValueSecondDerivative(double time) { return Value(); };


  virtual void setStartTime(double startTime) {
    startTime_ = startTime;
  }


  double getStartTime() const {
    return startTime_;
  }

  double getEndTime() const {
    return startTime_+duration_;
  }

  void setDuration(double duration) {
    duration_ = duration;
  }

  double getDuration() const {
    return duration_;
  }


 protected:
  double startTime_;
  double duration_;

};

} /* namespace robot_utils */
