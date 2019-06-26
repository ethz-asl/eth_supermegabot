/*!
* @file     ProfileSinusoid.hpp
* @author   Christian Gehring
* @date     April, 2015
* @version  1.0
* @ingroup  robot_utils
* @brief
*/

#pragma once

#include "robot_utils/schedule/Profile.hpp"
#include "kindr/Core"
#include "kindr/vectors/VectorBase.hpp"

namespace robot_utils {

namespace internal {
template<typename Profile_>
struct sinusoid_trait;
} // internal

template<typename Value_>
class ProfileSinusoid: public Profile<Value_>
{
 public:
  typedef Value_ Value;
  typedef Profile<Value_> Base;

  ProfileSinusoid(const Value& amplitude, const Value& offset, double frequency, double initialPhase, double duration) :
    Base(duration),
    amplitude_(amplitude),
    offset_(offset),
    frequency_(frequency),
    initialPhase_(initialPhase)
  {

  }

  virtual ~ProfileSinusoid() {

  }

  virtual void setStartTime(double startTime) {
//    double offset = this->startTime_ - startTime;
//    initialPhase_ += offset*2.0*M_PI*this->frequency_;
    initialPhase_ = -startTime*2.0*M_PI*this->frequency_;
    this->startTime_ = startTime;
  }

  virtual Value getValue(double time) {
    return internal::sinusoid_trait<Value_>::getValue(*this, time);

  }

  virtual Value getValueFirstDerivative(double time) {
    return internal::sinusoid_trait<Value_>::getValueFirstDerivative(*this, time);
  }

  virtual Value getValueSecondDerivative(double time) {
    return internal::sinusoid_trait<Value_>::getValueSecondDerivative(*this, time);
  }

  const Value& getAmplitude() const {
    return amplitude_;
  }
  const Value& getOffset() const {
    return offset_;
  }
  double getFrequency() const {
    return frequency_;
  }

  double getInitialPhase() const {
    return initialPhase_;
  }


 protected:
  Value amplitude_;
  Value offset_;
  double frequency_;
  double initialPhase_;
};


namespace internal {
template<typename Value_>
struct sinusoid_trait {
  inline static Value_ getValue(const ProfileSinusoid<Value_>& profile, double time) {
    return (profile.getAmplitude()*sin(2.0*M_PI*profile.getFrequency()*time + profile.getInitialPhase())+profile.getOffset());
  }
  inline static Value_ getValueFirstDerivative(const ProfileSinusoid<Value_>& profile, double time) {
//    if ( (time <= profile.getStartTime()) || (time >= profile.getEndTime()) ) {
//      return 0.0*profile.getAmplitude();
//    }
    return (2.0*M_PI*profile.getFrequency()*profile.getAmplitude()*cos(2.0*M_PI*profile.getFrequency()*time + profile.getInitialPhase()));
  }
  inline static Value_ getValueSecondDerivative(const ProfileSinusoid<Value_>& profile, double time) {
    return (  -(2.0*M_PI*profile.getFrequency())*(2.0*M_PI*profile.getFrequency())*profile.getAmplitude()*sin(2.0*M_PI*profile.getFrequency()*time + profile.getInitialPhase()));
  }
};
} // internal



} /* namespace robot_utils */
