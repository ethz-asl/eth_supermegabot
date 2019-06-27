/*!
 * @file    Imu.hpp
 * @author  Philipp Leemann
 * @date    Nov, 2017
 * @version 0.0
 *
 */
#pragma once

#include "kindr/Core"

#include "any_measurements/Time.hpp"


namespace any_measurements {

struct Imu
{
public:
    Imu() = default;

    Imu(const Time& time,
               const kindr::RotationQuaternionD& orientation,
               const kindr::LocalAngularVelocityD& angularVelocity,
               const kindr::Acceleration3D& linearAcceleration
    ):
            time_(time),
            orientation_(orientation),
            angularVelocity_(angularVelocity),
            linearAcceleration_(linearAcceleration)
    {
    }

    virtual ~Imu() = default;

public:
    Time time_;

    kindr::RotationQuaternionD orientation_;
    kindr::LocalAngularVelocityD angularVelocity_;
    kindr::Acceleration3D linearAcceleration_;
};


inline std::ostream& operator<<(std::ostream& os, const Imu& imu)
{
    return os << "Imu (Time: " << imu.time_ << ")"
              << "\n Orientation: " << imu.orientation_
              << "\n Acceleration: " << imu.linearAcceleration_
              << "\n Angular Velocity: " << imu.angularVelocity_;
}

} /* namespace any_measurements */
