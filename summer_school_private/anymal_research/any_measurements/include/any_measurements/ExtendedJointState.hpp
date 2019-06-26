/*!
 * @file    ExtendedJointState.hpp
 * @author  Fabian Tresoldi
 * @date    April 19, 2018
 * @version 0.0
 *
 */
#pragma once

#include "any_measurements/JointState.hpp"

namespace any_measurements {

struct ExtendedJointState : public JointState
{
 public:
    ExtendedJointState():
        JointState(),
        acceleration_(0.0)
    {
    }

    virtual ~ExtendedJointState() override = default;

 public:
    double acceleration_;
};

inline std::ostream& operator<<(std::ostream& os, const ExtendedJointState& extendedJointState)
{
    return os << "ExtendedJointState (Time: " << extendedJointState.time_ << ")"
              << "\n Position: " << extendedJointState.position_
              << "\n Velocity: " << extendedJointState.velocity_
              << "\n Acceleration: " << extendedJointState.acceleration_
              << "\n Effort: " << extendedJointState.effort_;
}


} /* namespace any_measurements */
