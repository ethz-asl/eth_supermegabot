/*
 * BmmPropagator.h
 *
 *  Created on: Jan 31, 2019
 *      Author: tim
 */

#ifndef INCLUDE_SMBSTATEESTIMATOR_BMMPROPAGATOR_H_
#define INCLUDE_SMBSTATEESTIMATOR_BMMPROPAGATOR_H_

#include <mutex>
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>
#include "smb_confusor/BaseMotionModel.h"

namespace smb_confusor {

//todo Should make a general propagator independent of the process model!
/**
 * Class to buffer BMM measurements and forward propagate the most recent state
 * estimates through the more recent measurements up to the current time.
 *
 * It is thread-safe, so measurements can be added from a separate thread
 * than that which periodically calls the update function.
 *
 * Written to support hard-real-time usage in Xenomai, avoiding things like std::cout.
 *
 * Uses BaseMotionModelParameters class to avoid dependency on a specific derived State implementation.
 */
class BmmPropagator {
public:
	BmmPropagator(int bufferSize = 1000);
	void addMeasurement(const WheelSpeeds& meas);
	void update(const BaseMotionModelParameters& stateUpdate);
	BaseMotionModelParameters propagate(double t);

	bool isBufferFull() { return measBuffer.full(); }
	bool isBufferEmpty() { return measBuffer.empty(); }

	//todo Better way to handle empty buffer?
	double frontMeasurementTime() {
		if (isBufferEmpty())
			return -1.0;

		return measBuffer.front().t();
	}
	double backMeasurementTime() {
		if (isBufferEmpty())
			return -1.0;

		return measBuffer.back().t();
	}

	WheelSpeeds getFrontMeasurement() { return measBuffer.front(); }
	WheelSpeeds getBackMeasurement() { return measBuffer.back(); }

protected:
	void forwardIntegrate(BaseMotionModelParameters& state, const WheelSpeeds& meas, double t_des);

	boost::circular_buffer<WheelSpeeds> measBuffer; //Store imu measurements since the time of the last update

	size_t prop_index = 0;
    BaseMotionModelParameters state_; //Propagated state. Gets updated to the time of the furthest imu measurement propagated through

	std::mutex mtx;
};

} // namespace smb_confusor

#endif /* INCLUDE_SMBSTATEESTIMATOR_BMMPROPAGATOR_H_ */
