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

#ifndef CONFUSION_TRAJECTORY_ALIGNMENTCOST_H
#define CONFUSION_TRAJECTORY_ALIGNMENTCOST_H

#include "confusion/utilities/distances.h"
#include "confusion/utilities/Pose.h"
#include "confusion/utilities/ceres_utils.h"

namespace confusion {

/**
 * A timed pose measurement
 */
struct TrajectoryPoint {
  TrajectoryPoint(const double& t, const Pose<double>& pose): t_(t), pose_(pose) { }

  double t() const { return t_; }
  Pose<double> &pose() { return pose_; }
  const Pose<double> &pose() const { return pose_; }

 protected:
  double t_;
  Pose<double> pose_;
};

/**
 * Computes the offsets to align two trajectories of frame i wrt frame w.
 * Quantities calibrated are:
 * T_wref_wmeas : Pose of the reference world frame with respect to the measurement world frame
 * T_imeas_iref : Pose of the reference target frame with respect to the measurement target frame
 * dt : Time offset between the two trajectories (t_ref = t_meas + dt)
 * scale : Scale difference between the two trajectories
 */
class TrajectoryAlignmentCost {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	TrajectoryAlignmentCost(const std::vector<TrajectoryPoint>& referenceTrajectory_,
			const double t_meas, const double w_trans_, const double w_rot_,
            const double refDropTime_,
			double* sum_error_=nullptr, double* error_out_=nullptr)
		: referenceTrajectory(referenceTrajectory_), t(t_meas), w_trans(w_trans_), w_rot(w_rot_),
          refDropTime(refDropTime_), sum_error(sum_error_), error_out(error_out_) {
#if defined OPT_DEBUG
		std::cout << "TrajectoryAlignmentCost created for t=" << t << std::endl;
#endif
	}

	template <typename T>
	bool operator()(T const* t_wmeas_imeas_, T const* q_wmeas_imeas_,
		T const* t_wref_wmeas_, T const* q_wref_wmeas_,
		T const* t_imeas_iref_, T const* q_imeas_iref_,
		T const* dt_, T const* scale_, T* residual_) const;

	const std::vector<TrajectoryPoint>& referenceTrajectory;
	const double t; //Time of the associated state instance
	const double w_trans;
	const double w_rot;
    const double refDropTime; //[sec] If the closeest igps measurement is this far from the pose measurement, ignore the cost function
	double* sum_error; //Sum the error vector over each iteration
	double* error_out; //Used to store the computed error for analysis
};

template <typename T>
bool TrajectoryAlignmentCost::operator()(T const* t_wmeas_imeas_, T const* q_wmeas_imeas_,
		T const* t_wref_wmeas_, T const* q_wref_wmeas_,
		T const* t_imeas_iref_, T const* q_imeas_iref_,
		T const* dt_, T const* scale_, T* residual_) const
{
	Eigen::Matrix<T,3,1> t_wmeas_imeas(t_wmeas_imeas_);
	t_wmeas_imeas *= scale_[0];
	Eigen::Quaternion<T> q_wmeas_imeas(q_wmeas_imeas_);
	Pose<T> T_wmeas_imeas(t_wmeas_imeas, q_wmeas_imeas);

	Eigen::Matrix<T,3,1> t_wref_wmeas(t_wref_wmeas_);
	Eigen::Quaternion<T> q_wref_wmeas(q_wref_wmeas_);
	Pose<T> T_wref_wmeas(t_wref_wmeas,q_wref_wmeas);

	Eigen::Matrix<T,3,1> t_imeas_iref(t_imeas_iref_);
	Eigen::Quaternion<T> q_imeas_iref(q_imeas_iref_);
	Pose<T> T_imeas_iref(t_imeas_iref,q_imeas_iref);

	//Get the estimated measurement
	Pose<T> T_wref_iref_est = T_wref_wmeas * T_wmeas_imeas * T_imeas_iref;

	//Get the interpolated ref measurement at time t+dt
	size_t index = 0;
	double dt_double;
	getDoubles(dt_,1,&dt_double);
	while (index < referenceTrajectory.size()-1 && referenceTrajectory[index+1].t() + dt_double < t) {
		++index;
	}

	if (index >= referenceTrajectory.size()-1) {
		std::cout << "Reached the end of the reference trajectory for state at t=" << t - dt_double << ". Setting the cost to zero." << std::endl;
		for (int i=0; i<6; ++i)
			residual_[i] = T(0.0);
	}
	else if (referenceTrajectory[index+1].t() - referenceTrajectory[index].t() < refDropTime) {
//		std::cout << "t=" << t-dt_double << "; t_igps_before=" << t_igps[index] << "; t_igps_after=" << t_igps[index+1] << std::endl;

		Pose<T> P0(referenceTrajectory[index].pose());
		Pose<T> P1(referenceTrajectory[index+1].pose());
		T t_interp = (T(t - referenceTrajectory[index].t()) - dt_[0]) / T(referenceTrajectory[index+1].t() - referenceTrajectory[index].t());

		Pose<T> T_wref_iref = P0.interpolate(t_interp, P1);

//		Pose<double> T_wref_iref_est_ = getDoublesPose(T_wref_iref_est);
//		Pose<double> T_wref_iref_ = getDoublesPose(T_wref_iref);
//		T_wref_iref_est_.print("T_wref_iref_est");
//		T_wref_iref_.print("T_wref_iref_meas");

		// Compute the residuals
		VectorDistance(T_wref_iref.trans.data(), T_wref_iref_est.trans.data(), residual_);
		QuatDistance(T_wref_iref.rot, T_wref_iref_est.rot, residual_+3);
	}
	else {
//std::cout << "Ignoring igps_cost at time " << t - dt_double << ". t_igps_before: " << t_igps[index] << "; t_igps_after: " << t_igps[index+1] << std::endl;
		for (int i=0; i<6; ++i)
			residual_[i] = T(0.0);
	}

	if (error_out) {
		getDoubles(residual_,6,error_out);
//std::cout << "Igps error: " << error_out[0] << "," << error_out[1] << "," << error_out[2] << "," << error_out[3] << "," << error_out[4] << "," << error_out[5] << "]" << std::endl;
	}

	residual_[0] *= T(w_trans);
	residual_[1] *= T(w_trans);
	residual_[2] *= T(w_trans);
	residual_[3] *= T(w_rot);
	residual_[4] *= T(w_rot);
	residual_[5] *= T(w_rot);

	if (sum_error) {
		double r[6];
		getDoubles(residual_,6,r);
		sum_error[0] += r[0]*r[0];
		sum_error[1] += r[1]*r[1];
		sum_error[2] += r[2]*r[2];
		sum_error[3] += r[3]*r[3];
		sum_error[4] += r[4]*r[4];
		sum_error[5] += r[5]*r[5];
	}

#if defined COST_DEBUG
	std::cout << "TrajectoryAlignmentCost = [" << residual_[0] << "," << residual_[1] << "," << residual_[2] << "," << residual_[3] << "," << residual_[4] << "," << residual_[5] << "]" << std::endl;
#endif
	return true;
}

} // namespace confusion

#endif // CONFUSION_TRAJECTORY_ALIGNMENTCOST_H
