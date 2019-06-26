/*
 * ImuWithTrigger.h
 *
 *  Created on: Sep 24, 2018
 *      Author: jelavice
 */

#pragma once

#include "any_measurements/Imu.hpp"

namespace any_measurements {

struct ImuWithTrigger: public Imu {

public:
	using Base = Imu;

	ImuWithTrigger() = default;

	ImuWithTrigger(const Time& time,
			const kindr::RotationQuaternionD& orientation,
			const kindr::LocalAngularVelocityD& angularVelocity,
			const kindr::Acceleration3D& linearAcceleration,
			bool triggerIndicator) :
			Base(time, orientation, angularVelocity, linearAcceleration),
			triggerIndicator_(triggerIndicator) {
	}

	~ImuWithTrigger() override = default;

	bool triggerIndicator_ = false;

};

inline std::ostream& operator<<(std::ostream& os, const ImuWithTrigger& imu)
{
    return os << "Imu (Time: " << imu.time_ << ")"
              << "\n Orientation: " << imu.orientation_
              << "\n Acceleration: " << imu.linearAcceleration_
              << "\n Angular Velocity: " << imu.angularVelocity_
			  << "\n tirgger indication: " << std::boolalpha << imu.triggerIndicator_;
}

} /* namespace*/
