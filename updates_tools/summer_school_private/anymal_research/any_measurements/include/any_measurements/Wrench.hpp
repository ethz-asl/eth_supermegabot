/*!
 * @file    Wrench.hpp
 * @author  Philipp Leemann
 * @date    Sep 19, 2016
 * @version 0.0
 *
 */
#pragma once

#include "kindr/Core"

#include "any_measurements/Time.hpp"

namespace any_measurements {

struct Wrench
{
 public:
    Wrench() = default;

    Wrench(const Time& time,
           const kindr::Force3D& force,
           const kindr::Torque3D& torque
    ):
        time_(time),
        wrench_(force, torque)
    {
    }

    Wrench(const Time& time,
            const kindr::WrenchD& wrench):
         time_(time),
         wrench_(wrench)
     {
     }

    virtual ~Wrench() = default;

 public:
    Time time_;
    kindr::WrenchD wrench_;
};

inline std::ostream& operator<<(std::ostream& os, const Wrench& wrench)
{
    return os << "Wrench (Time: " << wrench.time_ << ")"
              << "\n Wrench: " << wrench.wrench_;
}

} /* namespace any_measurements */
