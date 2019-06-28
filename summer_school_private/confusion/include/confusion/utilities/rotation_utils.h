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

#ifndef CONFUSION_ROTATION_UTILS_H_
#define CONFUSION_ROTATION_UTILS_H_

#include <Eigen/Geometry>
//#include <ceres/ceres.h>


namespace confusion {

inline Eigen::Vector3d quat_to_rpy(Eigen::Quaterniond quat) {
  Eigen::Matrix3d rotMat = quat.toRotationMatrix();
  return rotMat.eulerAngles(0, 1, 2);
}

inline Eigen::Quaterniond rpy_to_quat(double roll, double pitch, double yaw) {
  Eigen::Matrix3d rot_mat;
  rot_mat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  return Eigen::Quaterniond(rot_mat);
}

inline Eigen::Quaterniond rpy_to_quat(Eigen::Vector3d rpy) {
  Eigen::Matrix3d rot_mat;
  rot_mat = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());

  return Eigen::Quaterniond(rot_mat);
}

/**
 * Get the 3D gravity vector, given the pitch, roll, and magnitude of it.
 * This is used when optimizing only the roll and pitch of the gravity vector.
 * @param r [pitch, roll] of gravity [rad]
 * @param g_mag Magnitude of gravity
 * @return Gravity vector 3x[m/sec2]
 */
template<typename T>
inline Eigen::Matrix<T, 3, 1> gravityVec(T const *r, const T g_mag) {
  Eigen::Matrix<T, 3, 1> g;
  T cos_r1 = cos(r[1]);
  g(0) = sin(r[1]) * g_mag;
  g(1) = -sin(r[0]) * cos_r1 * g_mag;
  g(2) = cos(r[0]) * cos_r1 * g_mag;
  return g;
}

/**
 * Computes the derivative of the gravity vector wrt the roll and pitch angles
 * @param r
 * @param g_mag
 * @return
 */
inline Eigen::Matrix<double, 3, 2> gravityVec_jacob(double const *r, const double g_mag) {
  Eigen::Matrix<double, 3, 2> dg_dr;
  dg_dr << 0.0, cos(r[1]) * g_mag,
      -cos(r[0]) * cos(r[1]) * g_mag, sin(r[0]) * sin(r[1]) * g_mag,
      -sin(r[0]) * cos(r[1]) * g_mag, -cos(r[0]) * sin(r[1]) * g_mag;
  return dg_dr;
}

////Code copied from ceres/rotation.h to use eigen datatypes
//template<typename T>
//inline void QuatProduct(const Eigen::Quaternion<T>& z,
//		const Eigen::Quaternion<T>& w, Eigen::Quaternion<T>& zw) {
//	zw.w() = z.w() * w.w() - z.x() * w.x() - z.y() * w.y() - z.z() * w.z();
//	zw.x() = z.w() * w.x() + z.x() * w.w() + z.y() * w.z() - z.z() * w.y();
//	zw.y() = z.w() * w.y() - z.x() * w.z() + z.y() * w.w() + z.z() * w.x();
//	zw.z() = z.w() * w.z() + z.x() * w.y() - z.y() * w.x() + z.z() * w.w();
//}
//
template<typename T>
inline void QuatProduct_jacob_left(const Eigen::Quaternion<T> &z,
                                   const Eigen::Quaternion<T> &w, Eigen::Matrix<T, 4, 4> &dzw_dz) {
  //The output derivative rows and cols are ordered [w,x,y,z]!
  dzw_dz(0, 0) = w.w();
  dzw_dz(0, 1) = -w.x();
  dzw_dz(0, 2) = -w.y();
  dzw_dz(0, 3) = -w.z();
  dzw_dz(1, 0) = w.x();
  dzw_dz(1, 1) = w.w();
  dzw_dz(1, 2) = w.z();
  dzw_dz(1, 3) = -w.y();
  dzw_dz(2, 0) = w.y();
  dzw_dz(2, 1) = -w.z();
  dzw_dz(2, 2) = w.w();
  dzw_dz(2, 3) = w.x();
  dzw_dz(3, 0) = w.z();
  dzw_dz(3, 1) = w.y();
  dzw_dz(3, 2) = -w.x();
  dzw_dz(3, 3) = w.w();
}

template<typename T>
inline void QuatProduct_jacob_right(const Eigen::Quaternion<T> &z,
                                    const Eigen::Quaternion<T> &w, Eigen::Matrix<T, 4, 4> &dzw_dw) {
  //The output derivative rows and cols are ordered [w,x,y,z]!
  dzw_dw(0, 0) = z.w();
  dzw_dw(0, 1) = -z.x();
  dzw_dw(0, 2) = -z.y();
  dzw_dw(0, 3) = -z.z();
  dzw_dw(1, 0) = z.x();
  dzw_dw(1, 1) = z.w();
  dzw_dw(1, 2) = -z.z();
  dzw_dw(1, 3) = z.y();
  dzw_dw(2, 0) = z.y();
  dzw_dw(2, 1) = z.z();
  dzw_dw(2, 2) = z.w();
  dzw_dw(2, 3) = -z.x();
  dzw_dw(3, 0) = z.z();
  dzw_dw(3, 1) = -z.y();
  dzw_dw(3, 2) = z.x();
  dzw_dw(3, 3) = z.w();
}

//
//template <typename T>
//inline void QuatRotatePoint(const Eigen::Quaternion<T>& q, const Eigen::Matrix<T,3,1>& pt, Eigen::Matrix<T,3,1>& result) {
//	const T t2 =  q.w() * q.x();
//	const T t3 =  q.w() * q.y();
//	const T t4 =  q.w() * q.z();
//	const T t5 = -q.x() * q.x();
//	const T t6 =  q.x() * q.y();
//	const T t7 =  q.x() * q.z();
//	const T t8 = -q.y() * q.y();
//	const T t9 =  q.y() * q.z();
//	const T t1 = -q.z() * q.z();
//	result[0] = T(2) * ((t8 + t1) * pt[0] + (t6 - t4) * pt[1] + (t3 + t7) * pt[2]) + pt[0];  // NOLINT
//	result[1] = T(2) * ((t4 + t6) * pt[0] + (t5 + t1) * pt[1] + (t9 - t2) * pt[2]) + pt[1];  // NOLINT
//	result[2] = T(2) * ((t7 - t3) * pt[0] + (t2 + t9) * pt[1] + (t5 + t8) * pt[2]) + pt[2];  // NOLINT
//}
//
template<typename T>
inline void QuatRotatePoint_jacob_left(const Eigen::Quaternion<T> &q,
                                       const Eigen::Matrix<T, 3, 1> &pt, Eigen::Matrix<T, 3, 4> &dr_dq) {
//	result[0] = T(2) * ((-q.y()*q.y() + -q.z()*q.z()) * pt[0] + (q.x()*q.y() - q.w()*q.z()) * pt[1]   + (q.w()*q.y() + q.x()*q.z()) * pt[2]) + 	pt[0];  // NOLINT
//	result[1] = T(2) * ((q.w()*q.z() + q.x()*q.y()) * pt[0]   + (-q.x()*q.x() + -q.z()*q.z()) * pt[1] + (q.y()*q.z() - q.w()*q.x()) * pt[2]) + 	pt[1];  // NOLINT
//	result[2] = T(2) * ((q.x()*q.z() - q.w()*q.y()) * pt[0]   + (q.w()*q.x() + q.y()*q.z()) * pt[1]   + (-q.x()*q.x() + -q.y()*q.y()) * pt[2]) + pt[2];  // NOLINT

  //Jacobian is ordered [w,x,y,z]
  dr_dq(0, 0) = T(2) * (-q.z() * pt[1] + q.y() * pt[2]);
  dr_dq(0, 1) = T(2) * (q.y() * pt[1] + q.z() * pt[2]);
  dr_dq(0, 2) = T(2) * (-T(2) * q.y() * pt[0] + q.x() * pt[1] + q.w() * pt[2]);
  dr_dq(0, 3) = T(2) * (-T(2) * q.z() * pt[0] - q.w() * pt[1] + q.x() * pt[2]);
  dr_dq(1, 0) = T(2) * (q.z() * pt[0] - q.x() * pt[2]);
  dr_dq(1, 1) = T(2) * (q.y() * pt[0] - T(2) * q.x() * pt[1] - q.w() * pt[2]);
  dr_dq(1, 2) = T(2) * (q.x() * pt[0] + q.z() * pt[2]);
  dr_dq(1, 3) = T(2) * (q.w() * pt[0] - T(2) * q.z() * pt[1] + q.y() * pt[2]);
  dr_dq(2, 0) = T(2) * (-q.y() * pt[0] + q.x() * pt[1]);
  dr_dq(2, 1) = T(2) * (q.z() * pt[0] + q.w() * pt[1] - T(2) * q.x() * pt[2]);
  dr_dq(2, 2) = T(2) * (-q.w() * pt[0] + q.z() * pt[1] - T(2) * q.y() * pt[2]);
  dr_dq(2, 3) = T(2) * (q.x() * pt[0] + q.y() * pt[1]);
}

template<typename T>
inline void QuatRotatePoint_jacob_right(const Eigen::Quaternion<T> &q,
                                        const Eigen::Matrix<T, 3, 1> &pt, Eigen::Matrix<T, 3, 3> &dr_dpt) {
//	result[0] = T(2) * ((-q.y()*q.y() + -q.z()*q.z()) * pt[0] + (q.x()*q.y() - q.w()*q.z()) * pt[1] + 	(q.w()*q.y() + q.x()*q.z()) * pt[2]) + 	pt[0];  // NOLINT
//	result[1] = T(2) * ((q.w()*q.z() + q.x()*q.y()) * pt[0] + 	(-q.x()*q.x() + -q.z()*q.z()) * pt[1] + (q.y()*q.z() - q.w()*q.x()) * pt[2]) + 	pt[1];  // NOLINT
//	result[2] = T(2) * ((q.x()*q.z() - q.w()*q.y()) * pt[0] + 	(q.w()*q.x() + q.y()*q.z()) * pt[1] + 	(-q.x()*q.x() + -q.y()*q.y()) * pt[2]) + pt[2];  // NOLINT

  dr_dpt(0, 0) = T(1) - T(2) * (q.y() * q.y() + q.z() * q.z());
  dr_dpt(0, 1) = T(2) * (q.x() * q.y() - q.w() * q.z());
  dr_dpt(0, 2) = T(2) * (q.w() * q.y() + q.x() * q.z());
  dr_dpt(1, 0) = T(2) * (q.w() * q.z() + q.x() * q.y());
  dr_dpt(1, 1) = T(1) - T(2) * (q.x() * q.x() + q.z() * q.z());
  dr_dpt(1, 2) = T(2) * (q.y() * q.z() - q.w() * q.x());
  dr_dpt(2, 0) = T(2) * (q.x() * q.z() - q.w() * q.y());
  dr_dpt(2, 1) = T(2) * (q.w() * q.x() + q.y() * q.z());
  dr_dpt(2, 2) = T(1) - T(2) * (q.x() * q.x() + q.y() * q.y());
}

//Code copied from ceres/rotation.h to use eigen datatypes
template<typename T>
inline void quatToRotationMatrix(const Eigen::Quaternion<T> &q, Eigen::Matrix<T, 3, 3> &R) {
  // Make convenient names for elements of q.
  T a = q.w();
  T b = q.x();
  T c = q.y();
  T d = q.z();
  // This is not to eliminate common sub-expression, but to
  // make the lines shorter so that they fit in 80 columns! (from Ceres code...)
  T aa = a * a;
  T ab = a * b;
  T ac = a * c;
  T ad = a * d;
  T bb = b * b;
  T bc = b * c;
  T bd = b * d;
  T cc = c * c;
  T cd = c * d;
  T dd = d * d;

  R(0, 0) = aa + bb - cc - dd;
  R(0, 1) = T(2.0) * (bc - ad);
  R(0, 2) = T(2.0) * (ac + bd);
  R(1, 0) = T(2.0) * (ad + bc);
  R(1, 1) = aa - bb + cc - dd;
  R(1, 2) = T(2.0) * (cd - ab);
  R(2, 0) = T(2.0) * (bd - ac);
  R(2, 1) = T(2.0) * (ab + cd);
  R(2, 2) = aa - bb - cc + dd;
}

/**
 * Compute the derivative of a quaternion with respect to the rotation matrix
 * which was used to compute it.
 * @param R Rotation matrix used to get q
 * @param dq_dR
 */
template<typename T>
inline void calc_dq_dR(const Eigen::Matrix<T, 3, 3> &R, Eigen::Matrix<T, 4, 9> &dq_dR) {
  const T trace = R(0, 0) + R(1, 1) + R(2, 2);
  if (trace >= T(0.0)) {
    T c = T(1.0) + trace;
    T c1 = T(0.5) / sqrt(c);
    T c2 = T(0.5) * c1;
    T c3 = -c1 / T(2.0) / c;

    dq_dR(0, 0) = c2;
    dq_dR(0, 1) = T(0.0);
    dq_dR(0, 2) = T(0.0);
    dq_dR(0, 3) = T(0.0);
    dq_dR(0, 4) = c2;
    dq_dR(0, 5) = T(0.0);
    dq_dR(0, 6) = T(0.0);
    dq_dR(0, 7) = T(0.0);
    dq_dR(0, 8) = c2;

    dq_dR(1, 0) = (R(2, 1) - R(1, 2)) * c3;
    dq_dR(1, 1) = T(0.0);
    dq_dR(1, 2) = T(0.0);
    dq_dR(1, 3) = T(0.0);
    dq_dR(1, 4) = (R(2, 1) - R(1, 2)) * c3;
    dq_dR(1, 5) = -c1;
    dq_dR(1, 6) = T(0.0);
    dq_dR(1, 7) = c1;
    dq_dR(1, 8) = (R(2, 1) - R(1, 2)) * c3;

    dq_dR(2, 0) = (R(0, 2) - R(2, 0)) * c3;
    dq_dR(2, 1) = T(0.0);
    dq_dR(2, 2) = c1;
    dq_dR(2, 3) = T(0.0);
    dq_dR(2, 4) = (R(0, 2) - R(2, 0)) * c3;
    dq_dR(2, 5) = T(0.0);
    dq_dR(2, 6) = -c1;
    dq_dR(2, 7) = T(0.0);
    dq_dR(2, 8) = (R(0, 2) - R(2, 0)) * c3;

    dq_dR(3, 0) = (R(1, 0) - R(0, 1)) * c3;
    dq_dR(3, 1) = -c1;
    dq_dR(3, 2) = T(0.0);
    dq_dR(3, 3) = c1;
    dq_dR(3, 4) = (R(1, 0) - R(0, 1)) * c3;
    dq_dR(3, 5) = T(0.0);
    dq_dR(3, 6) = T(0.0);
    dq_dR(3, 7) = T(0.0);
    dq_dR(3, 8) = (R(1, 0) - R(0, 1)) * c3;
  } else {

    int i = 0;
    if (R(1, 1) > R(0, 0)) {
      i = 1;
    }

    if (R(2, 2) > R(i, i)) {
      i = 2;
    }

    const int j = (i + 1) % 3;
    const int k = (j + 1) % 3;
    T c = R(i, i) - R(j, j) - R(k, k) + 1.0;
    T c1 = T(0.5) / sqrt(c);
    T c2 = T(0.5) * c1;
    T c3 = -c1 / T(2.0) / c;

    //todo Is it possible to do this without making an extra copy?
    //First fill in with [0->9]=[ii,ij,ik,ji...]
    Eigen::Matrix<T, 4, 9> dq_dR_ijk;
    dq_dR_ijk(0, 0) = (R(k, j) - R(j, k)) * c3;
    dq_dR_ijk(0, 1) = T(0.0);
    dq_dR_ijk(0, 2) = T(0.0);
    dq_dR_ijk(0, 3) = T(0.0);
    dq_dR_ijk(0, 4) = T(-1.0) * (R(k, j) - R(j, k)) * c3;
    dq_dR_ijk(0, 5) = -c1;
    dq_dR_ijk(0, 6) = T(0.0);
    dq_dR_ijk(0, 7) = c1;
    dq_dR_ijk(0, 8) = T(-1.0) * (R(k, j) - R(j, k)) * c3;

    dq_dR_ijk(i + 1, 0) = c2;
    dq_dR_ijk(i + 1, 1) = T(0.0);
    dq_dR_ijk(i + 1, 2) = T(0.0);
    dq_dR_ijk(i + 1, 3) = T(0.0);
    dq_dR_ijk(i + 1, 4) = -c2;
    dq_dR_ijk(i + 1, 5) = T(0.0);
    dq_dR_ijk(i + 1, 6) = T(0.0);
    dq_dR_ijk(i + 1, 7) = T(0.0);
    dq_dR_ijk(i + 1, 8) = -c2;

    dq_dR_ijk(j + 1, 0) = (R(j, i) + R(i, j)) * c3;
    dq_dR_ijk(j + 1, 1) = c1;
    dq_dR_ijk(j + 1, 2) = T(0.0);
    dq_dR_ijk(j + 1, 3) = c1;
    dq_dR_ijk(j + 1, 4) = T(-1.0) * (R(j, i) + R(i, j)) * c3;
    dq_dR_ijk(j + 1, 5) = T(0.0);
    dq_dR_ijk(j + 1, 6) = T(0.0);
    dq_dR_ijk(j + 1, 7) = T(0.0);
    dq_dR_ijk(j + 1, 8) = T(-1.0) * (R(j, i) + R(i, j)) * c3;

    dq_dR_ijk(k + 1, 0) = (R(k, i) + R(i, k)) * c3;
    dq_dR_ijk(k + 1, 1) = T(0.0);
    dq_dR_ijk(k + 1, 2) = c1;
    dq_dR_ijk(k + 1, 3) = T(0.0);
    dq_dR_ijk(k + 1, 4) = T(-1.0) * (R(k, i) + R(i, k)) * c3;
    dq_dR_ijk(k + 1, 5) = T(0.0);
    dq_dR_ijk(k + 1, 6) = c1;
    dq_dR_ijk(k + 1, 7) = T(0.0);
    dq_dR_ijk(k + 1, 8) = T(-1.0) * (R(k, i) + R(i, k)) * c3;

    //Reorder terms to match the true i,j,k structure
    if (i == 1) {
      dq_dR.row(0) << dq_dR_ijk(0, 8), dq_dR_ijk(0, 6), dq_dR_ijk(0, 7), dq_dR_ijk(0, 2), dq_dR_ijk(0, 0), dq_dR_ijk(0,
                                                                                                                     1), dq_dR_ijk(
          0,
          5), dq_dR_ijk(0, 3), dq_dR_ijk(0, 4);
      dq_dR.row(1) << dq_dR_ijk(1, 8), dq_dR_ijk(1, 6), dq_dR_ijk(1, 7), dq_dR_ijk(1, 2), dq_dR_ijk(1, 0), dq_dR_ijk(1,
                                                                                                                     1), dq_dR_ijk(
          1,
          5), dq_dR_ijk(1, 3), dq_dR_ijk(1, 4);
      dq_dR.row(2) << dq_dR_ijk(2, 8), dq_dR_ijk(2, 6), dq_dR_ijk(2, 7), dq_dR_ijk(2, 2), dq_dR_ijk(2, 0), dq_dR_ijk(2,
                                                                                                                     1), dq_dR_ijk(
          2,
          5), dq_dR_ijk(2, 3), dq_dR_ijk(2, 4);
      dq_dR.row(3) << dq_dR_ijk(3, 8), dq_dR_ijk(3, 6), dq_dR_ijk(3, 7), dq_dR_ijk(3, 2), dq_dR_ijk(3, 0), dq_dR_ijk(3,
                                                                                                                     1), dq_dR_ijk(
          3,
          5), dq_dR_ijk(3, 3), dq_dR_ijk(3, 4);
    } else if (i == 2) {
      dq_dR.row(0) << dq_dR_ijk(0, 4), dq_dR_ijk(0, 5), dq_dR_ijk(0, 3), dq_dR_ijk(0, 7), dq_dR_ijk(0, 8), dq_dR_ijk(0,
                                                                                                                     6), dq_dR_ijk(
          0,
          1), dq_dR_ijk(0, 2), dq_dR_ijk(0, 0);
      dq_dR.row(1) << dq_dR_ijk(1, 4), dq_dR_ijk(1, 5), dq_dR_ijk(1, 3), dq_dR_ijk(1, 7), dq_dR_ijk(1, 8), dq_dR_ijk(1,
                                                                                                                     6), dq_dR_ijk(
          1,
          1), dq_dR_ijk(1, 2), dq_dR_ijk(1, 0);
      dq_dR.row(2) << dq_dR_ijk(2, 4), dq_dR_ijk(2, 5), dq_dR_ijk(2, 3), dq_dR_ijk(2, 7), dq_dR_ijk(2, 8), dq_dR_ijk(2,
                                                                                                                     6), dq_dR_ijk(
          2,
          1), dq_dR_ijk(2, 2), dq_dR_ijk(2, 0);
      dq_dR.row(3) << dq_dR_ijk(3, 4), dq_dR_ijk(3, 5), dq_dR_ijk(3, 3), dq_dR_ijk(3, 7), dq_dR_ijk(3, 8), dq_dR_ijk(3,
                                                                                                                     6), dq_dR_ijk(
          3,
          1), dq_dR_ijk(3, 2), dq_dR_ijk(3, 0);
    } else {
      dq_dR = dq_dR_ijk;
    }
  }
}

/**
 * Compute the quaternion resulting from rotating q by some delta, expressed in the tangent space of q.
 * This is used in the EigenQuaternionParameterization.
 * @param q
 * @param delta
 * @return Return q [boxPlus] delta
 */
template<typename T>
inline Eigen::Quaternion<T> quaternionBoxPlus(const Eigen::Quaternion<T> &q, const Eigen::Matrix<T, 3, 1> &delta) {
  Eigen::Quaternion<T> q_out;

//	Eigen::Matrix<T,3,1> delta = 0.5 * delta_; //We need the 0.5 factor for the delta to represent a physical rotational quantity
  const T delta_norm = delta.norm();

  if (delta_norm > T(0.0)) {
    const T sin_delta_by_delta = sin(T(0.5) * delta_norm) / delta_norm;
    Eigen::Quaternion<T> tmp(cos(0.5 * delta_norm),
                             sin_delta_by_delta * delta[0],
                             sin_delta_by_delta * delta[1],
                             sin_delta_by_delta * delta[2]);
    q_out = tmp * q;
  } else {
    q_out = q;
  }

  return q_out;
}

//Computes the derviative of the q', resulting from q' = q [boxplus] dq, wrt dq
inline Eigen::Matrix<double, 4, 3> quaternionBoxPlusJacob(const Eigen::Quaterniond &q) {
  Eigen::Matrix<double, 4, 3> dqout_ddq;

  //Note that this gives the jacobian in order [x,y,z,w] to reflect eigen's underlying data structure
  dqout_ddq << q.w(), q.z(), -q.y(),  // x
      -q.z(), q.w(), q.x(),  // y
      q.y(), -q.x(), q.w(),  // z
      -q.x(), -q.y(), -q.z();  // w

  dqout_ddq *= 0.5;

  return dqout_ddq;
}

//Computes the derviative of the q', resulting from q' = q [boxplus] dq, wrt dq.
//This one considers when the yaw orientation is fixed. Used in FixedYawParameterization.
inline Eigen::Matrix<double, 4, 2> quaternionBoxPlusJacobFixedYaw(const Eigen::Quaterniond &q) {
  Eigen::Matrix<double, 4, 2> dqout_ddq;

  //Note that this gives the jacobian in order [x,y,z,w] to reflect eigen's underlying data structure
  dqout_ddq << q.w(), q.z(),  // x
      -q.z(), q.w(),  // y
      q.y(), -q.x(),  // z
      -q.x(), -q.y();  // w

  dqout_ddq *= 0.5;

  return dqout_ddq;
}

//This was copied from Eigen/Quaternion.h because it didn't work with ceres::Jet datatypes.
template<typename T>
inline Eigen::Quaternion<T> slerp(const Eigen::Quaternion<T> q0, const Eigen::Quaternion<T> q1, const T t) {
  using std::acos;
  using std::sin;
  using std::abs;
  static const T one = T(1) - Eigen::NumTraits<T>::epsilon();
  T d = q0.dot(q1);
  T absD = abs(d);

  T scale0;
  T scale1;

  if (absD >= one) {
    scale0 = T(1) - t;
    scale1 = t;
  } else {
    // theta is the angle between the 2 quaternions
    T theta = acos(absD);
    T sinTheta = sin(theta);

    scale0 = sin((T(1) - t) * theta) / sinTheta;
    scale1 = sin((t * theta)) / sinTheta;
  }
  if (d < T(0)) scale1 = -scale1;

  return Eigen::Quaternion<T>(scale0 * q0.coeffs() + scale1 * q1.coeffs());
}

/**
 * Get the derivative of a quaternion, with the body moving with angular velocity omega, with respect to time.
 * The derivative is returned in order [x,y,z,w]
 *
 * \param q Orientation of the body
 * \param omega Angular velocity of the body
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> calulateDqDt(Eigen::Quaternion<T> q, Eigen::Matrix<T, 3, 1> omega) {
  Eigen::Quaternion<T> omega_(T(0.0), omega(0), omega(1), omega(2));

  //todo Could save a few multiplications by using the quatproduct jacobian directly so I can avoid the omega_.w computations
  Eigen::Quaternion<T> qq = q * omega_;
  Eigen::Matrix<T, 4, 1> dq_dt = T(0.5) * qq.coeffs();

  return dq_dt;
}

/**
 * Is used to transform the velocity of a body with respect to another point on that body.
 * Can be used as: w_vel_w_a = VT(t_w_b-t_w_a) * w_vel_w_b where a and b are
 * frames attached to a rigid body and the velocities are expressed with respect to
 * reference frame w.
 * Or w_vel_w_a = VT(q_w_a * t_a_b) * w_vel_w_b.
 *
 * \param w_pos_a_b Position of frame b with respect to frame a, expressed in frame w
 */
inline Eigen::Matrix<double, 6, 6> rigidBodyVelocityTransform(Eigen::Vector3d w_pos_a_b) {
  Eigen::Matrix<double, 6, 6> VT(Eigen::Matrix<double, 6, 6>::Identity());
  Eigen::Matrix<double, 3, 3> t_cross;
  t_cross << 0.0, -w_pos_a_b(2), w_pos_a_b(1),
      w_pos_a_b(2), 0.0, -w_pos_a_b(0),
      -w_pos_a_b(1), w_pos_a_b(0), 0.0;
  VT.block<3, 3>(3, 0) = t_cross;

  return VT;
}

}

#endif /* CONFUSION_ROTATION_UTILS_H_ */
