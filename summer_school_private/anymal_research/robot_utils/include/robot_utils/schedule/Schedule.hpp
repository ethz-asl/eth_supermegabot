/*!
* @file     Schedule.hpp
* @author   Christian Gehring
* @date     April, 2015
* @version  1.0
* @ingroup  robot_utils
* @brief
*/

#pragma once

#include "robot_utils/schedule/Profile.hpp"

#include <vector>
#include <memory>

namespace robot_utils {

template<typename Value_>
class Schedule
{
 public:
  typedef Value_ Value;
  Schedule(double startTime = 0.0) :
    startTime_(startTime)
  {

  }
  virtual ~Schedule() {

  }

  void addProfile(Profile<Value>* profile) {
    addProfile(std::shared_ptr<Profile<Value>>(profile));
  }

  void addProfile(std::shared_ptr<Profile<Value>> profile) {
    if (profiles_.empty()) {
      profile->setStartTime(startTime_);
      profiles_.push_back(profile);
    }
    else {
      profile->setStartTime(profiles_.back()->getEndTime());
      profiles_.push_back(profile);
    }
  }

  std::shared_ptr<Profile<Value>>& getProfile(double time) {
    if (profiles_.empty()) {
      throw std::runtime_error("Empty schedule!");
    }
    if (time < profiles_[0]->getEndTime()) {
      return profiles_[0];
    }
    if (time >= profiles_.back()->getStartTime()) {
      return profiles_.back();
    }

    for (auto& profile : profiles_) {
      if (time >= profile->getStartTime() && time < profile->getEndTime()) {
        return profile;
      }
    }

  }

  void clear() {
    profiles_.clear();
  }

  Value getValue(double time) {
    return getProfile(time)->getValue(time);
  }

  Value getValueFirstDerivative(double time) {
    return getProfile(time)->getValueFirstDerivative(time);
  }

  Value getValueSecondDerivative(double time) {
    return getProfile(time)->getValueSecondDerivative(time);
  }

  double getDuration() const {
    return this->getEndTime()-startTime_;
  }

  double getEndTime() const {
    return profiles_.back()->getEndTime();
  }

 protected:
  double startTime_;
  std::vector<std::shared_ptr<Profile<Value>>> profiles_;
};

} /* namespace robot_utils */
