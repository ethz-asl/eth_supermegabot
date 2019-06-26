/*
 * Copyright 2018 Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_CONFUSION_IMUPROPAGATOR_H_
#define INCLUDE_CONFUSION_IMUPROPAGATOR_H_

#include <mutex>
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>
#include <confusion/utilities/imu_utils.h>

namespace confusion {

/**
 * Class to buffer IMU measurements and forward propagate the most recent state
 * estimates through the more recent measurements up to the current time.
 *
 * It is thread-safe, so measurements can be added from a separate thread
 * than that which periodically calls the update function.
 *
 * Written to support hard-real-time usage in Xenomai, avoiding things like std::cout.
 *
 * Uses ImuStateParameters class to avoid dependency on a specific derived State implementation.
 */
class ImuPropagator {
public:
	ImuPropagator(int bufferSize = 1000);
	void addImuMeasurement(const ImuMeas& meas);
	void update(const ImuStateParameters& stateUpdate, const Eigen::Vector3d& g_t_update);
	void update(const ImuStateParameters& stateUpdate);
	ImuStateParameters propagate(double t);

	bool isBufferFull() { return imuBuffer.full(); }
	bool isBufferEmpty() { return imuBuffer.empty(); }

	//todo Better way to handle empty buffer?
	double frontMeasurementTime() {
		if (isBufferEmpty())
			return -1.0;

		return imuBuffer.front().t();
	}
	double backMeasurementTime() {
		if (isBufferEmpty())
			return -1.0;

		return imuBuffer.back().t();
	}

	ImuMeas getFrontMeasurement() { return imuBuffer.front(); }
	ImuMeas getBackMeasurement() { return imuBuffer.back(); }

protected:
	void forwardIntegrate(ImuStateParameters& state, const ImuMeas& meas, double t_des);

	boost::circular_buffer<ImuMeas> imuBuffer; //Store imu measurements since the time of the last update

	size_t prop_index = 0;
	Eigen::Vector3d g_t; //The 't' frame is the frame in which the state is expressed
	ImuStateParameters state; //Propagated state. Gets updated to the time of the furthest imu measurement propagated through

	std::mutex mtx;
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_IMUPROPAGATOR_H_ */
