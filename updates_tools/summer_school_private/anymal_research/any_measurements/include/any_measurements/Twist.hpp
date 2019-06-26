#pragma once

#include "kindr/Core"

#include "any_measurements/Time.hpp"

namespace any_measurements {

struct Twist
{
 public:
    Twist() = default;

    Twist(
        const Time& time,
        const kindr::TwistLocalD& twist
    ):
        time_(time),
        twist_(twist)
    {
    }


    virtual ~Twist() = default;

 public:
    Time time_;
    kindr::TwistLocalD twist_;
};

inline std::ostream& operator<<(std::ostream& os, const Twist& twist)
{
    return os << "Twist (Time: " << twist.time_ << ")"
              << "\n Twist: " << twist.twist_;
}

} /* namespace any_measurements */
