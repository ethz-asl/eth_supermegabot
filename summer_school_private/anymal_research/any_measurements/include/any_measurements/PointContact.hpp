/*!
 * @file    PointContact.hpp
 * @author  Christian Gehring
 * @date    Sep 19, 2016
 * @version 0.0
 *
 */
#pragma once
#include "any_measurements/Time.hpp"
#include "any_measurements/Wrench.hpp"

#include <kindr/Core>

namespace any_measurements {

class PointContact
{
 public:
  PointContact():
    time_(),
    wrench_(),
    position_(),
    twist_(),
    normal_(),
    state_(0)
  {
  }
    
  virtual ~PointContact() = default;

  Time time_;
  Wrench wrench_;
  kindr::Position3D position_;
  kindr::TwistLocalD twist_;
  kindr::VectorTypeless3D normal_;
  int state_;
};

inline std::ostream& operator<<(std::ostream& os, const PointContact& pointContact)
{
  return os << "PointContact (Time: " << pointContact.time_ << ")"
            << "\n Wrench: " << pointContact.wrench_
            << "\n Position: " << pointContact.position_
            << "\n Twist: " << pointContact.twist_
            << "\n Normal: " << pointContact.normal_
            << "\n State: " << pointContact.state_;
}

} /* namespace any_measurements */
