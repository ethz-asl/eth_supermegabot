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

#ifndef CONFUSION_DISTANCES_H_
#define CONFUSION_DISTANCES_H_

//#include "ceres/ceres.h"
#include <Eigen/Geometry>

namespace confusion {

/**
 * Compute the difference between two 3D vectors. Raw pointers
 * are passed to support Ceres's interface directly. If you are using Eigen types,
 * just provide xxx.data() as the input argument.
 * @param x1 Vector 1. If only one vector is a specialized type, this should not be the specialized one [m]
 * @param x2 Vector 2. If only one vector is a specialized type, this should be the specialized one [m]
 * @param d Resulting difference [m]x3
 */
template <typename T1,typename T2>
void VectorDistance(T2 const* x1, T1 const* x2, T1* d)
{
	d[0] = T1(x1[0]) - x2[0];
	d[1] = T1(x1[1]) - x2[1];
	d[2] = T1(x1[2]) - x2[2];
}

/**
 * Compute the difference between two 3D vectors. One Vector will be streched with scale. Raw
 * pointers are passed to support Ceres's interface directly. If you are using Eigen types, just
 * provide xxx.data() as the input argument.
 * @param x1 Vector 1. If only one vector is a specialized type, this should not be the specialized
 * one [m]
 * @param x2 Vector 2. If only one vector is a specialized type, this should be the specialized one
 * [m]
 * @param scale The scaling factor. In specialized type.
 * @param d Resulting difference [m]x3
 */
template <typename T1, typename T2>
void ScaledVectorDistance(T2 const* x1, T1 const* x2, T1 const* scale, T1* d) {
  d[0] = scale[0] * T1(x1[0]) - x2[0];
  d[1] = scale[0] * T1(x1[1]) - x2[1];
  d[2] = scale[0] * T1(x1[2]) - x2[2];
}

/**
 * Compute the rotational distance between two quaternions. This computes
 * a * b.inv and returns only the last three elements of the resulting quaternion,
 * making the small angle assumption such that the w element is negligibly small.
 * The user should pass eigen types to avoid errors with the underlying ordering of element in eigen quaternions.
 * @param a Quat 1. If only one vector is a specialized type, this should not be the specialized one
 * @param b Quat 2. If only one vector is a specialized type, this should be the specialized one
 * @param d Resulting distance [rad]x3
 */
template <typename T1, typename T2>
void QuatDistance(const Eigen::Quaternion<T2>& a, const Eigen::Quaternion<T1>& b, T1* d) {
	d[0] = T1(2.0) * (-T1(a.w())*b.x() + T1(a.x())*b.w() - T1(a.y())*b.z() + T1(a.z())*b.y());
	d[1] = T1(2.0) * (-T1(a.w())*b.y() + T1(a.x())*b.z() + T1(a.y())*b.w() - T1(a.z())*b.x());
	d[2] = T1(2.0) * (-T1(a.w())*b.z() - T1(a.x())*b.y() + T1(a.y())*b.x() + T1(a.z())*b.w());
}

template <typename T>
void QuatDistance(const Eigen::Quaternion<T>& a, const Eigen::Quaternion<T>& b, T* d) {
	d[0] = 2.0 * (-a.w()*b.x() + a.x()*b.w() - a.y()*b.z() + a.z()*b.y());
	d[1] = 2.0 * (-a.w()*b.y() + a.x()*b.z() + a.y()*b.w() - a.z()*b.x());
	d[2] = 2.0 * (-a.w()*b.z() - a.x()*b.y() + a.y()*b.x() + a.z()*b.w());
}

/**
 * Get the partial derivative of the distance between quaternions a and b (a-b) wrt quaternion a
 * @param a Quat 1 (This isn't actually used in the computation, but is passed to avoid errors in usage)
 * @param b Quat 2
 * @param j The derivative of d(qa,qb) with respect to qb. Dimension 3x4. Column order [w,x,y,z].
 */
template <typename T, typename T2>
void calc_de_dq_left(const Eigen::Quaternion<T2>& a, const Eigen::Quaternion<T>& b, Eigen::Matrix<T,3,4>& j) {
	j << 	-b.x(),  b.w(), -b.z(),  b.y(),
			-b.y(),  b.z(),  b.w(), -b.x(),
			-b.z(), -b.y(),  b.x(),  b.w();
	j *= T(2.0);
}


/**
 * Get the partial derivative of the distance between quaternions a and b (a-b) wrt quaternion b
 * @param a Quat 1
 * @param b Quat 2 (This isn't actually used in the computation, but is passed to avoid errors in usage)
 * @param j The derivative of d(qa,qb) with respect to qa. Dimension 3x4. Column order [w,x,y,z].
 */
template <typename T>
void calc_de_dq_right(const Eigen::Quaternion<T>& a, const Eigen::Quaternion<T>& b, Eigen::Matrix<T,3,4>& j) {
	j << 	a.x(), -a.w(), a.z(), -a.y(),
			a.y(), -a.z(), -a.w(), a.x(),
			a.z(), a.y(), -a.x(), -a.w();
	j *= T(2.0);
}
template <typename T>
Eigen::Matrix<T,3,4> calc_de_dq_right(const Eigen::Quaternion<T>& a, const Eigen::Quaternion<T>& b) {
	Eigen::Matrix<T,3,4> j;
	j << 	a.x(), -a.w(), a.z(), -a.y(),
			a.y(), -a.z(), -a.w(), a.x(),
			a.z(), a.y(), -a.x(), -a.w();
	j *= T(2.0);
	return j;
}

//This version does not make the small angle assumption
//It is slower than QuatDistance but gives more realistic distances as qdiff gets larger
template <typename T>
void QuatDistanceGlobal(const Eigen::Quaternion<T>& a, const Eigen::Quaternion<T>& b, T* d) {
	Eigen::Quaternion<T> qdiff = a * b.conjugate();

	//This ensures that the returned distance has norm < PI
	if (qdiff.w() < T(0))
		qdiff.coeffs() = -qdiff.coeffs();

	//Avoid rounding errors on w, which would generate a NaN in the acos below
	if (qdiff.w() > 1.0)
		qdiff.w() = T(1.0);
	else if (qdiff.w() < -1.0)
		qdiff.w() = T(-1.0);

	T c1 = 2.0 * acos(qdiff.w()); //todo If the quaternions aren't normalized, w might be > 1, generating nans. Explicitly check against this?
	T c2 = sqrt(qdiff.x()*qdiff.x() + qdiff.y()*qdiff.y() + qdiff.z()*qdiff.z());

	if (c2 < 1e-9) {
		d[0] = 0.0;
		d[1] = 0.0;
		d[2] = 0.0;
	}
	else {
		d[0] = qdiff.x() * c1 / c2;
		d[1] = qdiff.y() * c1 / c2;
		d[2] = qdiff.z() * c1 / c2;
	}
}

} //namespace confusion

#endif /* CONFUSION_DISTANCES_H_ */
