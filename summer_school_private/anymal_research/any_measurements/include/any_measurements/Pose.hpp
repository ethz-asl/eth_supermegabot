/*!
 * @file    Pose.hpp
 * @author  Philipp Leemann
 * @date    Sep 19, 2016
 * @version 0.0
 *
 */
#pragma once

#include "kindr/Core"

#include "any_measurements/Time.hpp"

namespace any_measurements {

struct Pose
{
 public:
    Pose() = default;

    Pose(const Time& time,
         const kindr::HomTransformQuatD& pose
    ):
        time_(time),
        pose_(pose)
    {
    }


    virtual ~Pose() = default;

 public:
    Time time_;
    kindr::HomTransformQuatD pose_;
};

inline std::ostream& operator<<(std::ostream& os, const Pose& pose)
{
    return os << "Pose (Time: " << pose.time_ << ")"
              << "\n Pose: " << pose.pose_;
}

} /* namespace any_measurements */
