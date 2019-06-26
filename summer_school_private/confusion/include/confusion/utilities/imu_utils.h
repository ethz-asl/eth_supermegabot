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

#ifndef INCLUDE_CONFUSION_IMU_UTILS_H_
#define INCLUDE_CONFUSION_IMU_UTILS_H_

#include <deque>
#include <Eigen/Core>
#include "confusion/utilities/Pose.h"
#include "confusion/utilities/rotation_utils.h"
#include "confusion/ProcessMeasurement.h"
#include "confusion/models/ImuMeas.h"

namespace confusion {


struct ImuStateParameters {
	const double t() { return t_; }

	void print(const std::string &prefix = "") const {
	  std::cout << "ImuStateParameters at t=" << t_ << "\n";
	  T_w_i_.print();
	  std::cout << "angVel: " << angVel_.transpose() << std::endl;
      std::cout << "linVel: " << linVel_.transpose() << std::endl;
      std::cout << "accelBias: " << accelBias_.transpose() << std::endl;
      std::cout << "gyroBias: " << gyroBias_.transpose() << std::endl;
	}

	double t_;
	Pose<double> T_w_i_;
	Eigen::Vector3d angVel_;
	Eigen::Vector3d linVel_;
	Eigen::Vector3d accelBias_;
	Eigen::Vector3d gyroBias_;
};

/**
 * Forward propagate an orientation q0 by a measured angular velocity w_i (with an additive bias b_g),
 * over the time intercal of length dt. Derivatives wrt q0, w_i, and b_g computed in parallel.
 * @param q0 Initial orientation
 * @param b_g gyroscope bias, expressed in body-fixed frame
 * @param w_i Measured angular velociy in body-fixed frame
 * @param dt Time interval [sec]
 * @param q1 Returns the orientation at time t0+dt (the propagated q is returned separately since the prior orientation needs to be used later in the integrate function)
 * @param dq1_dq0 Returns the derivative of the resulting orientation wrt the initial orientation
 * @param dq1_dwi Returns the derivative of the resulting orientation wrt the angular velocity measurement
 * @param dq1_dbg Returns the derivative of the resulting orientation wrt the gyro bias
 */
template <typename T>
void propagate_q(Eigen::Quaternion<T>& q0, const Eigen::Matrix<T,3,1>& b_g,
		const Eigen::Matrix<T,3,1>& w_i, const T& dt, Eigen::Quaternion<T>& q1,
		Eigen::Matrix<T,4,4>& dq1_dq0, Eigen::Matrix<T,4,3>& dq1_dwi,
		Eigen::Matrix<T,4,3>& dq1_dbg) {

	Eigen::Matrix<T,3,1> w = w_i-b_g;

	T theta = w.norm();

	if (theta > T(0.00000001)) {
		Eigen::Matrix<T,3,1> axis = w / theta;
		T c1 = theta * dt / T(2.0);
		T cos_c1 = cos(c1);
		T sin_c1 = sin(c1);

		Eigen::Quaternion<T> dq;
		dq.w() = cos_c1;
		dq.x() = sin_c1*axis.x();
		dq.y() = sin_c1*axis.y();
		dq.z() = sin_c1*axis.z();

		q1 = q0 * dq;

		QuatProduct_jacob_left(q0,dq,dq1_dq0);

		Eigen::Matrix<T,4,4> dq1_ddq;
		QuatProduct_jacob_right(q0,dq,dq1_ddq);

		//dq1_dwi
		Eigen::Matrix<T,4,3> ddq_dw;
		T c2 = dt / T(2.0) / theta;
		ddq_dw(0,0) = -c2 * sin_c1 * w(0);
		ddq_dw(0,1) = -c2 * sin_c1 * w(1);
		ddq_dw(0,2) = -c2 * sin_c1 * w(2);

		T term_1 = dt / theta / theta / T(2.0) * cos_c1;
		T term_2 = sin_c1 / theta;
		T term_3 = sin_c1 / theta / theta / theta;

		ddq_dw(1,0) = w(0)*w(0) * term_1 + term_2 - w(0)*w(0) * term_3;
		ddq_dw(1,1) = w(0)*w(1) * term_1 - w(0)*w(1) * term_3;
		//todo There was a value missing here!!!!! I filled it in just following the pattern of the others, but could check this more carefully
		ddq_dw(1,2) = w(0)*w(2) * term_1 - w(0)*w(2) * term_3;

		ddq_dw(2,0) = w(1)*w(0) * term_1 - w(1)*w(0) * term_3;
		ddq_dw(2,1) = w(1)*w(1) * term_1 + term_2 - w(1)*w(1) * term_3;
		ddq_dw(2,2) = w(1)*w(2) * term_1 - w(1)*w(2) * term_3;

		ddq_dw(3,0) = w(2)*w(0) * term_1 - w(2)*w(0) * term_3;
		ddq_dw(3,1) = w(2)*w(1) * term_1 - w(2)*w(1) * term_3;
		ddq_dw(3,2) = w(2)*w(2) * term_1 + term_2 - w(2)*w(2) * term_3;

		dq1_dwi = dq1_ddq * ddq_dw;

		//dq1_dbg
		dq1_dbg = T(-1.0) * dq1_dwi;
	}
	else {
//		std::cout << "Zero rotation from t0 to t1" << std::endl;
		Eigen::Quaternion<T> dq(T(1),T(0),T(0),T(0));
		q1 = q0;

		QuatProduct_jacob_left(q0,dq,dq1_dq0);

		Eigen::Matrix<T,4,4> dq1_ddq;
		QuatProduct_jacob_right(q0,dq,dq1_ddq);

		//dq1_dwi
		Eigen::Matrix<T,4,3> ddq_dw(Eigen::Matrix<T,4,3>::Zero());
		ddq_dw(1,0) = dt / T(2.0);
		ddq_dw(2,1) = dt / T(2.0);
		ddq_dw(3,2) = dt / T(2.0);

		dq1_dwi = dq1_ddq * ddq_dw;

		//dq1_dbg
		dq1_dbg = T(-1.0) * dq1_dwi;
	}
}


/**
 * Forward propagate an orientation q0 by a measured angular velocity w_i (with an additive bias b_g),
 * over the time intercal of length dt. Derivatives wrt q0, w_i, and b_g computed in parallel.
 * @param q0 Initial orientation
 * @param b_g gyroscope bias, expressed in body-fixed frame
 * @param w_i Measured angular velociy in body-fixed frame
 * @param dt Time interval [sec]
 * @param q1 Returns the orientation at time t0+dt (the propagated q is returned separately since the prior orientation needs to be used later in the integrate function)
 * @param ddq1_ddq0 Returns the derivative of the resulting orientation wrt the initial orientation
 * @param ddq1_dwi Returns the derivative of the resulting orientation wrt the angular velocity measurement
 * @param ddq1_dbg Returns the derivative of the resulting orientation wrt the gyro bias
 */
template <typename T>
void propagate_q_tangentSpace(Eigen::Quaternion<T>& q0, const Eigen::Matrix<T,3,1>& b_g,
		const Eigen::Matrix<T,3,1>& w_i, const T& dt, Eigen::Quaternion<T>& q1,
		Eigen::Matrix<T,3,3>& ddq1_ddq0, Eigen::Matrix<T,3,3>& ddq1_dwi,
		Eigen::Matrix<T,3,3>& ddq1_dbg) {

	Eigen::Matrix<T,3,1> w = w_i-b_g;

	T theta = w.norm();

	if (theta > T(0.00000001)) {
		Eigen::Matrix<T,3,1> axis = w / theta;
		T c1 = theta * dt / T(2.0);
		T cos_c1 = cos(c1);
		T sin_c1 = sin(c1);

		Eigen::Quaternion<T> dq;
		dq.w() = cos_c1;
		dq.x() = sin_c1*axis.x();
		dq.y() = sin_c1*axis.y();
		dq.z() = sin_c1*axis.z();

		q1 = q0 * dq;
	}
	else {
//		std::cout << "Zero rotation from t0 to t1" << std::endl;
		q1 = q0;
	}

	//Fill in Jacobians
	//Can ignore the R here because the noise is uniform in all directions??
	//Removing R fails the unit test but is that okay because it is only the resulting magnitude that matters?
	ddq1_ddq0.setIdentity();
	Eigen::Matrix<T,3,3> R;
	quatToRotationMatrix(q0, R);
	ddq1_dwi = R * T(dt / 2.0);
//	ddq1_dwi.setIdentity();
//	ddq1_dwi *= T(dt / 2.0);
	ddq1_dbg = ddq1_dwi * T(-1.0);
}

/**
 * Forward propagate an orientation q0 by a measured angular velocity w_i (with an additive bias b_g),
 * over the time intercal of length dt. No derviatives computed in parallel.
 * @param q0 Initial orientation
 * @param b_g gyroscope bias, expressed in body-fixed frame
 * @param w_i Measured angular velociy in body-fixed frame
 * @param dt Time interval [sec]
 * @param q1 Returns the orientation at time t0+dt (the propagated q is returned separately since the prior orientation needs to be used later in the integrate function)
 */
template <typename T>
void propagate_q_no_jacob(Eigen::Quaternion<T>& q0, const Eigen::Matrix<T,3,1>& b_g,
		const Eigen::Matrix<T,3,1>& w_i, const T& dt, Eigen::Quaternion<T>& q1) {

	Eigen::Matrix<T,3,1> w = w_i-b_g;

	T theta = w.norm();

	if (theta > T(0.00000001)) {
		Eigen::Matrix<T,3,1> axis = w / theta;
		T c1 = theta * dt / T(2.0);
		T cos_c1 = cos(c1);
		T sin_c1 = sin(c1);

		Eigen::Quaternion<T> dq;
		dq.w() = cos_c1;
		dq.x() = sin_c1*axis.x();
		dq.y() = sin_c1*axis.y();
		dq.z() = sin_c1*axis.z();

		q1 = q0 * dq;
	}
	else {
//		std::cout << "Zero rotation from t0 to t1" << std::endl;
		q1 = q0;
	}
}

//This one uses trapzoidal integration
template <typename T>
void propagate_q_no_jacob_trap(Eigen::Quaternion<T>& q0, const Eigen::Matrix<T,3,1>& b_g,
		const Eigen::Matrix<T,3,1>& w0_, const Eigen::Matrix<T,3,1>& w1_, const T& dt, Eigen::Quaternion<T>& q1) {

	Eigen::Matrix<T,3,1> w0 = w0_-b_g;
	Eigen::Matrix<T,3,1> w1 = w1_-b_g;

	//For t0 measurement
	T theta0 = w0.norm();
	Eigen::Matrix<T,3,1> axis0;
	T c10;
	if (theta0 > T(0.00000001)) {
		axis0 = w0 / theta0;
		c10 = theta0 * dt / T(2.0);
	}
	else {
		std::cout << "Zero rotation from t0 to t1" << std::endl;
//		q1 = q0;
		axis0.setZero();
		c10 = T(0.0);
	}

	//For t1 measurement
	T theta1 = w1.norm();
	Eigen::Matrix<T,3,1> axis1;
	T c11;
	if (theta1 > T(0.00000001)) {
		axis1 = w1 / theta1;
		c11 = theta1 * dt / T(2.0);
	}
	else {
		std::cout << "Zero rotation from t0 to t1" << std::endl;
//		q1 = q0;
		axis1.setZero();
		c11 = T(0.0);
	}

	//Average the two points
	T c1 = T(0.5) * (c10 + c11);
	Eigen::Matrix<T,3,1> axis = T(0.5) * (axis0 + axis1);

	T cos_c1 = cos(c1);
	T sin_c1 = sin(c1);

	Eigen::Quaternion<T> dq;
	dq.w() = cos_c1;
	dq.x() = sin_c1*axis.x();
	dq.y() = sin_c1*axis.y();
	dq.z() = sin_c1*axis.z();

	q1 = q0 * dq;
}

/**
 * Forward propagate the velocity of a body, expressed in a gravity-fixed frame
 * by linear acceleration measurement a_i (with an additive bias b_a and
 * expressed in a body-fixed frame) over the time dt. Does not compute jacobians.
 * @param v ///< Velocity at time t0
 * @param q ///< Orientation at time t0, expressed in world frame
 * @param b_a ///< Accelerometer bias, expressed in body-fixed frame [m/sec2]
 * @param a_i ///< Linear acceleration measured, expressed in body-fixed frame [m/sec2]
 * @param g_w ///< Gravity expressed in world frame [m/sec2]
 * @param dt ///< Time interval [sec]
 */
template <typename T>
void propagate_v_no_jacob(Eigen::Matrix<T,3,1>& v, const Eigen::Quaternion<T>& q,
		const Eigen::Matrix<T,3,1>& b_a, const Eigen::Matrix<T,3,1>& a_i,
		const Eigen::Matrix<T,3,1>& g_w, const T& dt) {
	//Propagate v
	Eigen::Matrix<T,3,1> a = a_i - b_a;
	Eigen::Matrix<T,3,1> a_w = q * a + g_w;
	v += a_w * dt;
}

//This one used trapezoidal integration
template <typename T>
void propagate_v_no_jacob_trap(Eigen::Matrix<T,3,1>& v, const Eigen::Quaternion<T>& q0, const Eigen::Quaternion<T>& q1,
		const Eigen::Matrix<T,3,1>& b_a, const Eigen::Matrix<T,3,1>& a_0, const Eigen::Matrix<T,3,1>& a_1,
		const Eigen::Matrix<T,3,1>& g_w, const T& dt) {
	Eigen::Matrix<T,3,1> a0 = a_0 - b_a;
	Eigen::Matrix<T,3,1> a_w0 = q0 * a0 + g_w;

	Eigen::Matrix<T,3,1> a1 = a_1 - b_a;
	Eigen::Matrix<T,3,1> a_w1 = q1 * a1 + g_w;

	Eigen::Matrix<T,3,1> a_w = T(0.5) * (a_w0 + a_w1);

	v += a_w * dt;
}

/**
 * Forward propagate the velocity of a body, expressed in a gravity-fixed frame
 * by linear acceleration measurement a_i (with an additive bias b_a and
 * expressed in a body-fixed frame) over the time dt. Computes the derivatives
 * of v1 wrt q, b_a, and a_i.
 * @param v ///< Velocity at time t0
 * @param q ///< Orientation at time t0, expressed in world frame
 * @param b_a ///< Accelerometer bias, expressed in body-fixed frame [m/sec2]
 * @param a_i ///< Linear acceleration measured, expressed in body-fixed frame [m/sec2]
 * @param g_w ///< Gravity expressed in world frame [m/sec2]
 * @param dt ///< Time interval [sec]
 * @param dv1_dq ///< Derivative of v1 wrt body orientation
 * @param dv1_dba ///< Derivative of v1 wrt accel bias
 * @param dv1_dai ///< Derivative of v1 wrt acceleration measurement
 */
template <typename T>
void propagate_v(Eigen::Matrix<T,3,1>& v, const Eigen::Quaternion<T>& q,
		const Eigen::Matrix<T,3,1>& b_a, const Eigen::Matrix<T,3,1>& a_i,
		const Eigen::Matrix<T,3,1>& g_w, const T& dt,
		Eigen::Matrix<T,3,4>& dv1_dq, Eigen::Matrix<T,3,3>& dv1_dba,
		Eigen::Matrix<T,3,3>& dv1_dai) {

	//Propagate v
//	propagate_v_no_jacob(v, q, b_a, a_i, g_w, dt);
	Eigen::Matrix<T,3,1> a = a_i - b_a;
	Eigen::Matrix<T,3,1> a_w = q * a + g_w;
	v += a_w * dt;

	//Get jacobians
	QuatRotatePoint_jacob_left(q,a,dv1_dq);
	dv1_dq *= dt;

	QuatRotatePoint_jacob_right(q,a,dv1_dai);
	dv1_dai *= dt;

	dv1_dba = T(-1.0) * dv1_dai;
}



/**
 * Forward propagate the velocity of a body, expressed in a gravity-fixed frame
 * by linear acceleration measurement a_i (with an additive bias b_a and
 * expressed in a body-fixed frame) over the time dt. Computes the derivatives
 * of v1 wrt q, b_a, a_i, and g_w.
 * @param v ///< Velocity at time t0
 * @param q ///< Orientation at time t0, expressed in world frame
 * @param b_a ///< Accelerometer bias, expressed in body-fixed frame [m/sec2]
 * @param a_i ///< Linear acceleration measured, expressed in body-fixed frame [m/sec2]
 * @param g_w ///< Gravity expressed in world frame [m/sec2]
 * @param dt ///< Time interval [sec]
 * @param dv1_dq ///< Derivative of v1 wrt body orientation
 * @param dv1_dba ///< Derivative of v1 wrt accel bias
 * @param dv1_dai ///< Derivative of v1 wrt acceleration measurement
 * * @param dv1_dgw ///< Derivative of v1 wrt gravity
 */
template <typename T>
void propagate_v(Eigen::Matrix<T,3,1>& v, const Eigen::Quaternion<T>& q,
		const Eigen::Matrix<T,3,1>& b_a, const Eigen::Matrix<T,3,1>& a_i,
		const Eigen::Matrix<T,3,1>& g_w, const T& dt,
		Eigen::Matrix<T,3,4>& dv1_dq, Eigen::Matrix<T,3,3>& dv1_dba,
		Eigen::Matrix<T,3,3>& dv1_dai, Eigen::Matrix<T,3,3>& dv1_dgw) {

	propagate_v(v, q, b_a, a_i, g_w, dt, dv1_dq, dv1_dba, dv1_dai);

	dv1_dgw.setIdentity();
	dv1_dgw *= dt;
}




/**
 * Forward propagate the pose of a body due to the specified IMU measurements
 * over time dt. Also propagate the covariance of the body pose and compute
 * the derivative of the resulting pose with respect to the previous one.
 * @param p_ Position of the body at t0 [m]
 * @param q_ Orientation of the body at t0
 * @param v_ Velocity of the body at t0 [m/sec]
 * @param ba_ Accelerometer bias, expressed in body-fixed frame [m/sec2]
 * @param bg_ Gyro bias, expressed in body-fixed frame [rad/sec]
 * @param meas_ IMU measurement at t0
 * @param gw_ Gravity, expressed in world-fixed frame
 * @param dt_ Time interval
 * @param cov_x_ Covariance of the state, to be propagated from t0 to t0+dt
 * @param dx1_dx0 Derivative of the state at t1 wrt the state at t0. The state is [p,q,v,ba,bg]
 * @param cov_imu_meas_ Covariance of the imu measurement, ordered [w, a, bg, ba].
 */
template <typename T>
void integrate(Eigen::Matrix<T,3,1>& p_, Eigen::Quaternion<T>& q_,
		Eigen::Matrix<T,3,1>& v_, Eigen::Matrix<T,3,1>& ba_,
		Eigen::Matrix<T,3,1>& bg_, const std::shared_ptr<ProcessMeasurement> meas_,
		const Eigen::Matrix<T,3,1>& gw_, const T& dt_,
		Eigen::Matrix<T,16,16>& cov_x_,  //Used for calculating the stiffness matrix
		Eigen::Matrix<T,16,16>& dx1_dx0, //Used for Jacobian calculation
		const Eigen::Matrix<T,12,12>& cov_imu_meas_) {

	if (dt_ < T(0.0)) {
		std::cout << "imu integration was given a negative dt! exiting early. dt=" << dt_ << std::endl;
		return;
	}

	auto u_ = std::dynamic_pointer_cast<ImuMeas>(meas_);

	//Propagate p
	p_ += v_ * dt_;

	//Propagate q -- Note that q_ is not overwritten until the end because it is needed in propagate_v!
	Eigen::Quaternion<T> q_prop;
	Eigen::Matrix<T,3,1> omega_i(T(u_->w_.x()), T(u_->w_.y()), T(u_->w_.z()));
	Eigen::Matrix<T,4,4> dq1_dq0;
	Eigen::Matrix<T,4,3> dq1_dwi;
	Eigen::Matrix<T,4,3> dq1_dbg;
	propagate_q(q_, bg_, omega_i, dt_, q_prop,
			dq1_dq0, dq1_dwi, dq1_dbg);

	//Propagate v
	Eigen::Matrix<T,3,1> a_i(T(u_->a_.x()), T(u_->a_.y()), T(u_->a_.z()));
	Eigen::Matrix<T,3,4> dv1_dq0;
	Eigen::Matrix<T,3,3> dv1_dba;
	Eigen::Matrix<T,3,3> dv1_dai;
	propagate_v(v_, q_, ba_, a_i, gw_, dt_,
			dv1_dq0, dv1_dba, dv1_dai);

	//Build the full dx1_dx0
	Eigen::Matrix<T,16,16> dxj_dxi(Eigen::Matrix<T,16,16>::Identity());
	dxj_dxi.template block<3,3>(0,7) = Eigen::Matrix<T,3,3>::Identity();
	dxj_dxi.template block<3,3>(0,7) *= dt_;
	dxj_dxi.template block<4,4>(3,3) = dq1_dq0;
	dxj_dxi.template block<4,3>(3,13) = dq1_dbg;
	dxj_dxi.template block<3,4>(7,3) = dv1_dq0;
	dxj_dxi.template block<3,3>(7,10) = dv1_dba;

	//Build the full dx1_dz0. Note z is ordered [w a bg ba]!
	//todo Should change the order of z to [a w ba bg] to be consistent with state ordering
	Eigen::Matrix<T,16,12> dx1_dz0(Eigen::Matrix<T,16,12>::Zero());
	dx1_dz0.template block<4,3>(3,0) = dq1_dwi;
	dx1_dz0.template block<3,3>(7,3) = dv1_dai;
	dx1_dz0.template block<3,3>(10,9) = Eigen::Matrix<T,3,3>::Identity() * dt_;
	dx1_dz0.template block<3,3>(13,6) = Eigen::Matrix<T,3,3>::Identity() * dt_;

	//Propagate the state covariance
	cov_x_ = dxj_dxi * cov_x_ * dxj_dxi.transpose() +
			dx1_dz0 * cov_imu_meas_ * dx1_dz0.transpose();

	//Propagate the jacobians
	dx1_dx0 = dxj_dxi * dx1_dx0;

	//Dont forget to update q!
	q_ = q_prop;
}

/**
 * Forward propagate the pose of a body due to the specified IMU measurements
 * over time dt. Also propagate the covariance of the body pose and compute
 * the derivative of the resulting pose with respect to the previous one.
 * Also compute dx1_dgw, for use when gravity is also optimized online.
 * @param p_ Position of the body at t0 [m]
 * @param q_ Orientation of the body at t0
 * @param v_ Velocity of the body at t0 [m/sec]
 * @param ba_ Accelerometer bias, expressed in body-fixed frame [m/sec2]
 * @param bg_ Gyro bias, expressed in body-fixed frame [rad/sec]
 * @param meas_ IMU measurement at t0
 * @param gw_ Gravity, expressed in world-fixed frame
 * @param dt_ Time interval
 * @param cov_x_ Covariance of the state, to be propagated from t0 to t0+dt
 * @param dx1_dx0 Derivative of the state at t1 wrt the state at t0. The state is [p,q,v,ba,bg]
 * @param dx1_dgw_ Derivative of the state at t1 wrt the gravity vector. The state is [p,q,v,ba,bg]
 * @param cov_imu_meas_ Covariance of the imu measurement, ordered [w, a, bg, ba].
 */
template <typename T>
void integrate_opt_gravity(Eigen::Matrix<T,3,1>& p_, Eigen::Quaternion<T>& q_,
		Eigen::Matrix<T,3,1>& v_, Eigen::Matrix<T,3,1>& ba_,
		Eigen::Matrix<T,3,1>& bg_, const std::shared_ptr<ProcessMeasurement> meas_,
		const Eigen::Matrix<T,3,1>& gw_, const T& dt_,
		Eigen::Matrix<T,16,16>& cov_x_,  //Used for calculating the stiffness matrix
		Eigen::Matrix<T,16,16>& dx1_dx0, //Used for Jacobian calculation
		Eigen::Matrix<T,16,3>& dx1_dgw_,  //Used for Jacobian calculation
		const Eigen::Matrix<T,12,12>& cov_imu_meas_) {

	if (dt_ < T(0.0)) {
		std::cout << "imu integration was given a negative dt! exiting early. dt=" << dt_ << std::endl;
		return;
	}

	auto u_ = std::dynamic_pointer_cast<ImuMeas>(meas_);

	//Propagate p
	p_ += v_ * dt_;

	//Propagate q -- Note that q_ is not overwritten until the end because it is needed in propagate_v!
	Eigen::Quaternion<T> q_prop;
	Eigen::Matrix<T,3,1> omega_i(T(u_->w_.x()), T(u_->w_.y()), T(u_->w_.z()));
	Eigen::Matrix<T,4,4> dq1_dq0;
	Eigen::Matrix<T,4,3> dq1_dwi;
	Eigen::Matrix<T,4,3> dq1_dbg;
	propagate_q(q_, bg_, omega_i, dt_, q_prop,
			dq1_dq0, dq1_dwi, dq1_dbg);

	//Propagate v
	Eigen::Matrix<T,3,1> a_i(T(u_->a_.x()), T(u_->a_.y()), T(u_->a_.z()));
	Eigen::Matrix<T,3,4> dv1_dq0;
	Eigen::Matrix<T,3,3> dv1_dba;
	Eigen::Matrix<T,3,3> dv1_dai;
	Eigen::Matrix<T,3,3> dv1_dgw;
	propagate_v(v_, q_, ba_, a_i, gw_, dt_,
			dv1_dq0, dv1_dba, dv1_dai, dv1_dgw);

	//Build the full dx1_dx0
	Eigen::Matrix<T,16,16> dxj_dxi(Eigen::Matrix<T,16,16>::Identity());
	dxj_dxi.template block<3,3>(0,7) = Eigen::Matrix<T,3,3>::Identity();
	dxj_dxi.template block<3,3>(0,7) *= dt_;
	dxj_dxi.template block<4,4>(3,3) = dq1_dq0;
	dxj_dxi.template block<4,3>(3,13) = dq1_dbg;
	dxj_dxi.template block<3,4>(7,3) = dv1_dq0;
	dxj_dxi.template block<3,3>(7,10) = dv1_dba;

	Eigen::Matrix<T,16,3> dxj_dgw(Eigen::Matrix<T,16,3>::Zero());
	dxj_dgw.template block<3,3>(7,0) = dv1_dgw;

	//Build the full dx1_dz0. Note z is ordered [w a bg ba]!
	//todo Should change the order of z to [a w ba bg] to be consistent with state ordering
	Eigen::Matrix<T,16,12> dx1_dz0(Eigen::Matrix<T,16,12>::Zero());
	dx1_dz0.template block<4,3>(3,0) = dq1_dwi;
	dx1_dz0.template block<3,3>(7,3) = dv1_dai;
	dx1_dz0.template block<3,3>(10,9) = Eigen::Matrix<T,3,3>::Identity() * dt_;
	dx1_dz0.template block<3,3>(13,6) = Eigen::Matrix<T,3,3>::Identity() * dt_;

	//Propagate the state covariance
	cov_x_ = dxj_dxi * cov_x_ * dxj_dxi.transpose() +
			dx1_dz0 * cov_imu_meas_ * dx1_dz0.transpose();

	//Propagate the jacobians
	dx1_dx0 = dxj_dxi * dx1_dx0;
	dx1_dgw_ = dxj_dgw + dxj_dxi * dx1_dgw_;

	//Dont forget to update q!
	q_ = q_prop;
}

template <typename T>
bool integrate_no_jacob(Eigen::Matrix<T,3,1>& p_, Eigen::Quaternion<T>& q_,
		Eigen::Matrix<T,3,1>& v_, Eigen::Matrix<T,3,1>& ba_,
		Eigen::Matrix<T,3,1>& bg_, const ImuMeas* meas_,
		const Eigen::Matrix<T,3,1>& gw_, const T& dt_) {

	if (dt_ < T(0.0)) {
		std::cout << "imu integration was given a negative dt! exiting early. dt=" << dt_ << std::endl;
		return false;
	}

	//Propagate p
	p_ += v_ * dt_;

	//Propagate q -- Note that q_ is not overwritten until the end because it is needed in propagate_v!
	Eigen::Quaternion<T> q_prop;
	Eigen::Matrix<T,3,1> omega_i(T(meas_->w_.x()), T(meas_->w_.y()), T(meas_->w_.z()));
	propagate_q_no_jacob(q_, bg_, omega_i, dt_, q_prop);

	//Propagate v
	Eigen::Matrix<T,3,1> a_i(T(meas_->a_.x()), T(meas_->a_.y()), T(meas_->a_.z()));
	propagate_v_no_jacob(v_, q_, ba_, a_i, gw_, dt_);

	//Dont forget to update q!
	q_ = q_prop;

	return true;
}

template <typename T>
bool integrate_no_jacob(Eigen::Matrix<T,3,1>& p_, Eigen::Quaternion<T>& q_,
		Eigen::Matrix<T,3,1>& v_, Eigen::Matrix<T,3,1>& ba_,
		Eigen::Matrix<T,3,1>& bg_, const std::shared_ptr<ProcessMeasurement>& meas_,
		const Eigen::Matrix<T,3,1>& gw_, const T& dt_) {
	auto u_ = std::dynamic_pointer_cast<ImuMeas>(meas_);

	return integrate_no_jacob(p_, q_, v_, ba_, bg_, u_.get(),  gw_, dt_);
}

//This is used in createNextState in ImuState and HyaState
void forwardPropagateImuMeas(const std::deque<std::shared_ptr<ProcessMeasurement>>& imuMeasurements,
		const double& t_in, const double& t_des, const Eigen::Vector3d& g_w, Pose<double>& T_w_i,
		Eigen::Vector3d& linVel, Eigen::Vector3d& accelBias, Eigen::Vector3d& gyroBias) {
	//Do some timing checks
//	if (imuMeasurements.front()->t() > t_in) {
	//TODO I made this check less aggressive because of the logic problem in adding process measurements in MeasMan
	if (imuMeasurements.front()->t()-t_in > 0.01) {
		std::cout << "forwardPropagateImuMeas: The current state is older than the first IMU measurement. t_imu_front-t_state=" <<
				imuMeasurements.front()->t() - t_in <<  "; t_imu_back-t_imu_front=" <<
				imuMeasurements.back()->t() - imuMeasurements.front()->t() << std::endl;
	}
	if (t_des > imuMeasurements.back()->t() + 0.01) {
		std::cout << "forwardPropagateImuMeas: The last imu measurement is " << t_des-imuMeasurements.back()->t() <<
				" sec past the lastest IMU measurement. t_des=" << t_des << "; t_imu_back:" <<
				imuMeasurements.back()->t() << std::endl;
	}

	std::deque<std::shared_ptr<ProcessMeasurement>>::const_iterator imu_current = imuMeasurements.begin();

	//Move to the imu measurement at or before t_in
	while ((imu_current+1) != imuMeasurements.end() && (*(imu_current+1))->t() <= t_in)
		++imu_current;

	//Propagate from ti_ to t_imu_1
	if ((imu_current+1) != imuMeasurements.end() && (*imu_current)->t() <= t_in) {
		integrate_no_jacob(T_w_i.trans, T_w_i.rot, linVel, accelBias, gyroBias,
			*imu_current, g_w, (*(imu_current+1))->t() - t_in);
		++imu_current;
	}
	else {
		//This when there is no imu measurement preceeding t_
		integrate_no_jacob(T_w_i.trans, T_w_i.rot, linVel, accelBias, gyroBias,
			*imu_current, g_w, (*imu_current)->t() - t_in);
	}
//std::cout << "cov_x out 1:\n" << cov_x << std::endl << std::endl;
	//Iterate through the imu measurements
	while ((imu_current+1) != imuMeasurements.end() && (*(imu_current+1))->t() <= t_des) {
		integrate_no_jacob(T_w_i.trans, T_w_i.rot, linVel, accelBias, gyroBias,
			*imu_current, g_w, (*(imu_current+1))->t() - (*imu_current)->t());
		++imu_current;
	}

	//Propagate the last chunk of time. Don't do anything else if the last IMU measurement is directly at t_des
	if ((*imu_current)->t() < t_des) {
		integrate_no_jacob(T_w_i.trans, T_w_i.rot, linVel, accelBias, gyroBias,
			*imu_current, g_w, t_des - (*imu_current)->t());
	}
}

//Estimate the IMU attitude from the accelerometer measurement. Assumes that the
//true acceleration is small relative to gravity. The yaw orientation is unobservable
//from the accelerometer measurement alone.
Eigen::Quaterniond estimateImuOrientation(Eigen::Vector3d accel) {
	return Eigen::Quaterniond::FromTwoVectors(accel,Eigen::Vector3d(0,0,1));
}

} //namespace confusion

#endif /* INCLUDE_CONFUSION_IMU_UTILS_H_ */
