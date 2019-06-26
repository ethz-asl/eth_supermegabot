/*
 * BmmPropagator.cpp
 *
 *  Created on: Jan 31, 2019
 *      Author: tim
 */

#include "smb_confusor/BmmPropagator.h"

namespace smb_confusor {

BmmPropagator::BmmPropagator(int bufferSize) {
	measBuffer.set_capacity(bufferSize);
}

void BmmPropagator::addMeasurement(const WheelSpeeds& meas) {
	std::lock_guard<std::mutex> lockGuard(mtx);

	if (!measBuffer.empty() && meas.t() < measBuffer.back().t())
		printf("WARNING: Measurement added to the propagator which jumps back in time! t_back=%f, t_new=%f", measBuffer.back().t(), meas.t());
        else if (meas.t() == measBuffer.back().t())
          return; // Don't add duplicate measurements. smb_lowlevel is currently adding each measurement multiple times.

	//We need to move the meas index if the buffer is already full
	if (measBuffer.full()) {
		--prop_index;
		if (prop_index < 0) {
			printf("BmmPropagator state has fallen behind the entire measBuffer. Something is very wrong! \n");
		}
	}

	//Push back the new measurement
	measBuffer.push_back(meas);
//printf("Bmm measurement added to BmmPropagator at %6.6f\n", meas.t());
}

//Convenience function to integrate and update the time of the state
void BmmPropagator::forwardIntegrate(BaseMotionModelParameters &state, const WheelSpeeds& meas, double t_des) {
	if (fabs(state.t - t_des) < 1e-10) {
		printf("Very small dt=%f. Skipping the integration step. \n", fabs(state.t - t_des));
		return;
	}

    forwardIntegrateBaseMotionModel(state.T_w_b, meas, t_des - state.t);

	state.t = t_des;
}

void BmmPropagator::update(const BaseMotionModelParameters& stateUpdate) {
	state_ = stateUpdate;

	//Remove old measurements
	std::lock_guard<std::mutex> lockGuard(mtx);

	while (measBuffer.size() > 1 && measBuffer[1].t() <= stateUpdate.t) {
		measBuffer.pop_front();
	}
	prop_index = 0;

//	if (state_.t > measBuffer.back().t())
//		printf("WARNING: BmmPropagator updated to a time newer than the received measurements. meas_front "
//				"is %f sec behind and meas_back is %f sec behind\n", state_.t - measBuffer.front().t(), state_.t - measBuffer.back().t());
//	printf("BmmPropagator updated to t=%6.6f; t_imu_front=%6.6f; t_imu_back=%6.6f; with %zu measurements \n",
//			state_.t, measBuffer[prop_index].t(), measBuffer.back().t(), measBuffer.size());
}

//Forward propagate through the imu measurements up to time t_des
//Return the estimated state at time t_des
//Note that the internally stored state is only updated up to the time of the closest preceding imu measurement
BaseMotionModelParameters BmmPropagator::propagate(double t_des) {
//std::cout << "Propagating to t_des=" << t_des << ". t_state=" << state_.t <<
//		". t_imu_front=" << measBuffer.front().t() << " t_imu_prop=" << measBuffer[prop_index].t() << ". t_imu_back=" << measBuffer.back().t() << std::endl;
//state_.print("state before");
//measBuffer[prop_index].print();
	std::lock_guard<std::mutex> lockGuard(mtx);

	if (t_des < measBuffer[prop_index].t()) {
		printf("ERROR: BmmPropagator requested to propagate backwards in time. t_des=%f; t_imu=%f \n", t_des, measBuffer[prop_index].t());
		return state_;
	}

	//Do some sanity checks
	if (measBuffer.front().t() > state_.t) {
		printf("The BmmPropagator state is older than the first measurement. t_imu_front-t_state=%f; t_imu_back-t_imu_front=%f \n",
		    measBuffer.front().t() - state_.t, measBuffer.back().t() - measBuffer.front().t());
	}
	if ((measBuffer[prop_index].t() - state_.t) > 1e-10) {
		printf("BmmPropagator state behind the current imu meas!!?? t_imu=%f, t_state=%f \n", measBuffer[prop_index].t(), state_.t);
	}

	//Move to the time of the next meas if starting between measurement times
	if (state_.t != measBuffer[prop_index].t() &&
			prop_index != measBuffer.size()-1 &&
			measBuffer[prop_index+1].t() < t_des) {
//std::cout << "Update front: from " << state.t << " to " << measBuffer[prop_index+1].t() <<
//		" using meas at t_imu=" << measBuffer[prop_index].t() << std::endl;
		forwardIntegrate(state_, measBuffer[prop_index], measBuffer[prop_index+1].t());
		++prop_index;
	}

	//Propagate through the measurements
	while (prop_index != measBuffer.size()-1 && measBuffer[prop_index+1].t() <= t_des) {
//std::cout << "Update middle: from " << state_.t << " to " << measBuffer[prop_index+1].t() <<
//		" using meas at t_imu=" << measBuffer[prop_index].t() << std::endl;
		forwardIntegrate(state_, measBuffer[prop_index], measBuffer[prop_index+1].t());
		++prop_index;
	}

	//Propagate the last chunk of time. Don't move the current index and don't update the internal state.
	BaseMotionModelParameters state_out = state_;
	if (t_des < measBuffer[prop_index].t()) {
//std::cout << "Last chunk: from " << state_.t << " to " << t_des <<
//		" using meas at t_imu=" << measBuffer[prop_index].t() << std::endl;
//std::cout << "diff: " << t_des - measBuffer[prop_index].t() << std::endl;
		forwardIntegrate(state_out, measBuffer[prop_index], t_des);
	}

	return state_out;
}

} // namespace smb_confusor
