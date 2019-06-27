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


#ifndef CONFUSION_UTILS_POSE_H_
#define CONFUSION_UTILS_POSE_H_

#include <iostream>
#include <Eigen/Geometry>
#include <confusion/utilities/rotation_utils.h>
#include <confusion/utilities/distances.h>

namespace confusion {

/**
 * @class Pose
 *
 * @brief A class to represent the 3D pose of a frame or object
 *
 * The class uses passive rotation and gives the same behavior as if using
 * 4x4 transformation matrices.
 * Designed for use in Ceres Sovler.
 * Templated on type for use with auto-differentiation.
 *
 * In this documentation the following conventions are used: T_a_b is the pose of b expressed in frame a.
 * a_t_b is the position of b expressed in frame a.
 */
template <typename T>
class Pose
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	/**
	 * Construct from raw elements of the pose (translation and quaternion)
	 * @param x [m]
	 * @param y [m]
	 * @param z [m]
	 * @param qw
	 * @param qx
	 * @param qy
	 * @param qz
	 */
	Pose(T x, T y, T z, T qw, T qx, T qy, T qz)
		: trans(x, y, z), rot(qw, qx, qy, qz) {}

	/**
	 * Construct from eigen types
	 * @param trans [m] x 3
	 * @param rot [quaternion]
	 */
	Pose(const Eigen::Matrix<T,3,1>& trans, const Eigen::Quaternion<T> rot)
		: trans(trans), rot(rot) {}

//	Pose(const geometry_msgs::Pose& msg)
//		: trans(T(msg.position.x),T(msg.position.y),T(msg.position.z)),
//		  rot(T(msg.orientation.w), T(msg.orientation.x), T(msg.orientation.y), T(msg.orientation.z)) {}

	/**
	 * Construct a Pose from a homogeneous transform
	 * @param transformation - 4x4 transformation matrix
	 */
	Pose(const Eigen::Matrix<T, 4, 4>& transformation) {
		trans = transformation.template block<3,1>(0,3) / transformation(3,3);
		Eigen::Matrix<T,3,3> rotMat = transformation.block(0,0,3,3);
		rot = Eigen::Quaternion<T>(rotMat);
	}

	Pose()
		: trans(T(0.0), T(0.0), T(0.0)), rot(T(1.0), T(0.0), T(0.0), T(0.0)) {}
		
	//Allows for easy conversion from double to ceres::Jet types, but not vice-versa
	template <typename OtherT>
	Pose(const Pose<OtherT>& p) 
		: trans(T(p.trans.x()), T(p.trans.y()), T(p.trans.z())),
		rot(T(p.rot.w()), T(p.rot.x()), T(p.rot.y()), T(p.rot.z()))
	{}
	
	//Allows for easy conversion from double to ceres::Jet types, but not vice-versa
	template <typename OtherT>
	Pose& operator=(const Pose<OtherT>& p)
	{
		if(this == &p)
			return *this;
		
		trans = Eigen::Matrix<T,3,1>(T(p.trans.x()), T(p.trans.y()), T(p.trans.z()));
		rot = Eigen::Quaternion<T>(T(p.rot.w()), T(p.rot.x()), T(p.rot.y()), T(p.rot.z()));
		
		return *this;
	}

	/**
	 * Get the inverse of the pose. rot = q.conjugate(), trans = -q.conjugate() * trans.
	 *
	 * inverse(T_A_B) = T_B_A
	 *
	 * @return
	 */
	Pose<T> inverse() const
	{
		Pose<T> output;
		output.rot = rot.conjugate();
		output.trans = output.rot * trans;
		output.trans *= T(-1.0);

		return output;
	}

	/**
	 * Standard multiplication of transformations. T_A_C = T_A_B * T_B_C.
	 * @param p_in
	 * @return p_out = this * p_in
	 */
	Pose<T> operator*(const Pose<T>& p_in) const
	{
		Pose<T> p_out;
		p_out.rot = rot * p_in.rot;

		p_out.trans = trans + rot * p_in.trans;

		return p_out;
	}

	/**
	 * Tranform a point in space to the frame of the pose. p_A = T_A_B * p_B
	 * @param p_in
	 * @return p_out = this * p_in
	 */
	Eigen::Matrix<T,3,1> operator*(const Eigen::Matrix<T,3,1>& p_in) const
	{
		Eigen::Matrix<T,3,1> p_out;
		p_out = rot*p_in;
		p_out += trans;
	
		return p_out;
	}
	
	/**
	 * Utility for quickly printing the contents of a pose
	 */
	void print() const
	{
		printf("t: [%f,%f,%f]; q: [%f,%f,%f,%f]\n",
				trans[0], trans[1], trans[2],
				rot.w(), rot.x(), rot.y(), rot.z());
	}

	/**
	 * Utility for quickly printing the contents of a pose
	 * @param prefix This is printed before the contents of the pose
	 */
	void print(std::string prefix) const
	{
		printf("%s --- t: [%f,%f,%f]; q: [%f,%f,%f,%f]\n",  prefix.c_str(),
				trans[0], trans[1], trans[2],
				rot.w(), rot.x(), rot.y(), rot.z());
	}

	/**
	 * Get a motion transform for the pose. This can be used to express the velocity of a rigid body about a different frame attached to that body.
	 * Agrees with Robcogen conventions. The 6D velocity vector is NOT a spatial velocity, just a stacked 3D angular velocity and 3D linear velocity.
	 *
	 * @return MT Can be used as a_vel_w_a = MT(T_a_b) * b_vel_w_b
	 */
	Eigen::Matrix<T,6,6> toMotionTransform() {
		Eigen::Matrix<T,6,6> MT;

		Eigen::Matrix<T,3,3> R = rot.toRotationMatrix();

		Eigen::Matrix<T,3,3> t_cross;
		t_cross << 	T(0), -trans(2), trans(1),
					trans(2), T(0), -trans(0),
					-trans(1), trans(0), T(0);

		MT.block(0,0,3,3) = R;
		MT.block(0,3,3,3) = Eigen::Matrix<T,3,3>::Zero();
		MT.block(3,0,3,3) = t_cross * R;
		MT.block(3,3,3,3) = R;

		return MT;
	}

	//This rotates a velocity vector (a stacked angular velocity and linear velocity).
	//It simply rotates each component of the velocity, so it should NOT be used
	//with spatial velocities, as are typically used in RBD dynamics calculations.
	Eigen::Matrix<double,6,1> rotateVelocityVector(Eigen::Matrix<double,6,1> v_in) const {
		Eigen::Matrix<double,6,1> v_out;
		v_out.head<3>() = rot * v_in.head<3>();
		v_out.tail<3>() = rot * v_in.tail<3>();
		return v_out;
	}

//	//Gives the spatial vector transformation matrix along with its derviative, given the velocity of the object
//	//The returned matrix T transforms spatial velocities in the base frame to spatial velocities in the object frame
//	//todo Not really clear what this is doing. Use with caution!
//	void toSpatialVectorTransform(Eigen::Matrix<T,6,6>& T_in, Eigen::Matrix<T,6,6>& Td, const Eigen::Matrix<T,6,1>& xd) const {
//
//		Eigen::Matrix<T,3,3> w_cross;
//		w_cross << 	0, -xd(2), xd(1),
//					xd(2), 0, -xd(0),
//					-xd(1), xd(0), 0;
//
//		Eigen::Matrix<T,3,3> Rinv = (rot.inverse()).toRotationMatrix();
//
//		T_in.block(0,0,3,3) = Rinv;
//		T_in.block(0,3,3,3) = Eigen::Matrix<T,3,3>::Zero();
//		T_in.block(3,0,3,3) = Eigen::Matrix<T,3,3>::Zero(); //-Rinv * d_cross;
//		T_in.block(3,3,3,3) = Rinv;
//
//		//Robcogen Jacobians do not compute spatial velocities, but stacked rot and trans velocity vectors!
//		Td.block(0,0,3,3) = Rinv * w_cross.transpose(); //dRotMat = w_cross * RotMat, so dRotMatInv = RotMatInv * w_crossTranspose
//		Td.block(0,3,3,3) = Eigen::Matrix<T,3,3>::Zero();
//		Td.block(3,0,3,3) = Eigen::Matrix<T,3,3>::Zero(); //-Rinv * w_cross.transpose() * d_cross - Rinv * v_cross;
//		Td.block(3,3,3,3) = Rinv * w_cross.transpose();
//	}

	/**
	 * Convenience function to set the elemenets of the pose from a 4x4 homogeneous transform
	 * @param transformation Transformation matrix (4x4)
	 */
	void setFromHomogTrans(const Eigen::Matrix<T, 4, 4>& transformation) {
		trans = transformation.block(0,3,3,1) / transformation(3,3);
		rot = Eigen::Quaterniond(transformation.block(0,0,3,3));
	}

	//Interpolate between two poses. t is in [0,1], where 0 would return this,
	//and 1 would return P1. Quaternions interpolated using slerp.
	Pose<T> interpolate(T t, Pose<T> P1) const {
		Pose<T> Pout;
		Pout.trans = trans + (P1.trans - trans) * t;
		Pout.rot = slerp(rot, P1.rot, t);
		return Pout;
	}

	Eigen::Matrix<T,4,4> toTransformationMatrix() const {
		Eigen::Matrix<T,4,4> tmat;
		tmat.template topLeftCorner<3,3>() = rot.toRotationMatrix();
		tmat.template topRightCorner<3,1>() = trans;
		tmat.row(3) << 0, 0, 0, 1;
		return tmat;
	}

	//Get the distance in tangent space between two poses. Ordered [drot; dtrans]
	//d = T_w_a * T_w_b^-1
	Eigen::Matrix<T,6,1> distance(Pose<T> P1) const {
		Eigen::Matrix<T,6,1> d;
		QuatDistance(rot,P1.rot,d.data());
		VectorDistance(trans.data(),P1.trans.data(),d.data()+3);
		return d;
	}

	//Get the distance in global space between two poses. Ordered [drot; dtrans]
	//This requires more computation than distance, but is accurate in the presence of large rotational differences
	Eigen::Matrix<T,6,1> distanceGlobal(Pose<T> P1) const {
		Eigen::Matrix<T,6,1> d;
		QuatDistanceGlobal(rot,P1.rot,d.data());
		VectorDistance(trans.data(),P1.trans.data(),d.data()+3);
		return d;
	}

public:
	Eigen::Matrix<T,3,1> trans; ///< position
	Eigen::Quaternion<T> rot; ///< orientation (stored as a quaternion)
};


//Get the pose of the first tag seen, given the current estimate of the orientation of the world frame.
//The world frame is rotated in the x and y directions (rotation in the z-direction is unobservable).
//todo Find a better place for this
template <typename T>
Pose<T> getTagPoseFromRot(const Pose<T>& T_w_t_init, const Eigen::Matrix<T,2,1>& firstTagRot) {
	T phi_roll = firstTagRot(0) / T(2.0);
	T phi_pitch = firstTagRot(1) / T(2.0);
	Eigen::Quaternion<T> dq_roll(cos(phi_roll),sin(phi_roll),T(0.0),T(0.0));
	Eigen::Quaternion<T> dq_pitch(cos(phi_pitch),T(0.0),sin(phi_pitch),T(0.0));
	return Pose<T>(T_w_t_init.trans, dq_roll * dq_pitch * T_w_t_init.rot);
}


}


#endif // CONFUSION_UTILS_POSE_H_
