/*!
 * @file    JointState.hpp
 * @author  Philipp Leemann
 * @date    Sep 19, 2016
 * @version 0.0
 *
 */
#pragma once

#include "any_measurements/Time.hpp"

namespace any_measurements {

struct JointState
{
 public:
    JointState():
        time_(),
        position_(0.0),
        velocity_(0.0),
        effort_(0.0)
    {
    }

    virtual ~JointState() = default;

 public:
    Time time_;

    double position_;
    double velocity_;
    double effort_;
};

inline std::ostream& operator<<(std::ostream& os, const JointState& jointState)
{
    return os << "JointState (Time: " << jointState.time_ << ")"
              << "\n Position: " << jointState.position_
              << "\n Velocity: " << jointState.velocity_
              << "\n Effort: " << jointState.effort_;
}


} /* namespace any_measurements */
