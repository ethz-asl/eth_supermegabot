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

#include "confusion/utilities/ImuPropagator.h"

namespace confusion {

ImuPropagator::ImuPropagator(int bufferSize) {
	imuBuffer.set_capacity(bufferSize);

	//Initialize to the nominal gravity. The user can specify different values through the update function.
	g_t(0) = 0.0; g_t(1) = 0.0; g_t(2) = -9.80665;
}

void ImuPropagator::addImuMeasurement(const ImuMeas& meas) {
	std::lock_guard<std::mutex> lockGuard(mtx);

	if (!imuBuffer.empty() && meas.t() < imuBuffer.back().t())
		printf("WARNING: Imu measurement added to the propagator which jumps back in time! t_back=%f, t_new=%f",
		        imuBuffer.back().t(), meas.t());

	//We need to move the imu index if the buffer is already full
	if (imuBuffer.full()) {
		--prop_index;
		if (prop_index < 0) {
			printf("ImuPropagator state has fallen behind the entire imuBuffer. Something is very wrong! \n");
		}
	}

	//Push back the new measurement
	imuBuffer.push_back(meas);
}

//Convenience function to integrate and update the time of the state
//todo Strange to pass that state and not g_t. What is proper way to do this?
void ImuPropagator::forwardIntegrate(ImuStateParameters& state_, const ImuMeas& meas, double t_des) {
	if (fabs(state_.t_ - t_des) < 1e-10) {
		printf("Very small dt=%f. Skipping the integration step. \n", fabs(state_.t_ - t_des));
		return;
	}

	if (!integrate_no_jacob(state_.T_w_i_.trans, state_.T_w_i_.rot,
			state_.linVel_, state_.accelBias_, state_.gyroBias_,
			&meas, g_t, t_des - state_.t_))
		printf("Integrating from %f to %f using imu meas at t_imu=%f failed \n", state_.t_, t_des, meas.t());

	state_.t_ = t_des;
}


//Update the state from a time in the past and forward propagate through the
//measurements up to the previously propagated time.
void ImuPropagator::update(const ImuStateParameters& stateUpdate, const Eigen::Vector3d& g_t_update) {
	g_t = g_t_update;

	update(stateUpdate);
}

void ImuPropagator::update(const ImuStateParameters& stateUpdate) {
  state = stateUpdate;

  //Remove old measurements
  std::lock_guard<std::mutex> lockGuard(mtx);

  while (imuBuffer.size() > 1 && imuBuffer[1].t() <= stateUpdate.t_) {
      imuBuffer.pop_front();
  }
  prop_index = 0;

  if (state.t_ > imuBuffer.back().t())
      printf("WARNING: ImuPropagator updated to a time newer than the received IMU measurements. imu_front "
              "is %f sec behind and imu_back is %f sec behind\n", state.t_ - imuBuffer.front().t(), state.t_ - imuBuffer.back().t());
//	printf("ImuPropagator updated to t=%f; t_imu_front=%f; t_imu_back=%f \n",
//			state.t_, imuBuffer[prop_index].t(), imuBuffer.back().t());
}

//Forward propagate through the imu measurements up to time t_des
//Return the estimated state at time t_des
//Note that the internally stored state is only updated up to the time of the closest preceding imu measurement
ImuStateParameters ImuPropagator::propagate(double t_des) {
//std::cout << "Propagating to t_des=" << t_des << ". t_state=" << state.t_ <<
//		". t_imu_front=" << imuBuffer.front().t() << " t_imu_prop=" << imuBuffer[prop_index].t() << ". t_imu_back=" << imuBuffer.back().t() << std::endl;
//state.print("state before");
//imuBuffer[prop_index].print();
	std::lock_guard<std::mutex> lockGuard(mtx);

	if (t_des < imuBuffer[prop_index].t()) {
		printf("ERROR: ImuPropagator requested to propagate backwards in time. t_des=%f; t_imu=%f \n", t_des, imuBuffer[prop_index].t());
		return state;
	}

	//Do some sanity checks
	if (imuBuffer.front().t() > state.t_) {
		printf("The ImuPropagator state is older than the first IMU measurement. t_imu_front-t_state=%f; t_imu_back-t_imu_front=%f \n", imuBuffer.front().t() - state.t_, imuBuffer.back().t() - imuBuffer.front().t());
	}
	if ((imuBuffer[prop_index].t() - state.t_) > 1e-10) { //This was triggering due to numerical inaccuracies
		printf("ImuPropagator state behind the current imu meas!!?? t_imu=%f, t_state=%f \n", imuBuffer[prop_index].t(), state.t_);
	}

	//Move to the time of the next imu meas if starting between measurement times
	if (state.t_ != imuBuffer[prop_index].t() &&
			prop_index != imuBuffer.size()-1 &&
			imuBuffer[prop_index+1].t() < t_des) {
//std::cout << "Update front: from " << state.t_ << " to " << imuBuffer[prop_index+1].t() <<
//		" using meas at t_imu=" << imuBuffer[prop_index].t() << std::endl;
		forwardIntegrate(state, imuBuffer[prop_index], imuBuffer[prop_index+1].t());
		++prop_index;
	}

	//Propagate through the measurements
	while (prop_index != imuBuffer.size()-1 && imuBuffer[prop_index+1].t() <= t_des) {
//std::cout << "Update middle: from " << state.t_ << " to " << imuBuffer[prop_index+1].t() <<
//		" using meas at t_imu=" << imuBuffer[prop_index].t() << std::endl;
		forwardIntegrate(state, imuBuffer[prop_index], imuBuffer[prop_index+1].t());
		++prop_index;
	}

	//Update the angular velocity
	state.angVel_ = state.T_w_i_.rot * (imuBuffer[prop_index].w_ - state.gyroBias_);

	//Propagate the last chunk of time. Don't move the current index and don't update the internal state.
	ImuStateParameters state_out = state;
	if (t_des < imuBuffer[prop_index].t()) {
//std::cout << "Last chunk: from " << state.t_ << " to " << t_des <<
//		" using meas at t_imu=" << imuBuffer[prop_index].t() << std::endl;
//std::cout << "diff: " << t_des - imuBuffer[prop_index].t() << std::endl;
		forwardIntegrate(state_out, imuBuffer[prop_index], t_des);
	}

	return state_out;
}

} // namespace confusion
