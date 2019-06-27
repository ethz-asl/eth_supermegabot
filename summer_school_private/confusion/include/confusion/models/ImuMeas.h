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

#ifndef CONFUSION_IMUMEAS_H
#define CONFUSION_IMUMEAS_H

#include <Eigen/Core>
#include "confusion/ProcessMeasurement.h"

namespace confusion {

struct ImuCalibration {
  bool imu_use_loss_func = false;
  double imu_loss_coeff = 0.0;
  Eigen::Matrix<double,12,12> cov_imu_nominal_;
  double gravityMagnitude_;
  Eigen::Vector3d g_w_;
};


class ImuMeas : public ProcessMeasurement {
 public:
  ImuMeas(): ProcessMeasurement(0, 0.0) { }

  ImuMeas(double t, Eigen::Vector3d& a_in, Eigen::Vector3d& w_in,
          ImuCalibration* imuCalibration = nullptr, Eigen::Vector2d* gravity_rot = nullptr, int measurementType = 0):
      ProcessMeasurement(measurementType, t), a_(a_in), w_(w_in),
      imuCalibration_(imuCalibration), gravity_rot_(gravity_rot) { }

  ImuMeas& operator=(const ImuMeas& meas) {
    if(this == &meas)
      return *this;

    t_ = meas.t();
    measType_ = meas.measType();
    a_ = meas.a_;
    w_ = meas.w_;
    imuCalibration_ = meas.imuCalibration_;
    gravity_rot_ = meas.gravity_rot_;

    return *this;
  }

  //Used for first state initialization
  Eigen::Quaterniond estimateInitialOrientation() {
    return Eigen::Quaterniond::FromTwoVectors(a_,Eigen::Vector3d(0,0,1));
  }

  Eigen::Vector3d a_; ///< Linear acceleration [m/sec2] typically expressed in IMU body frame
  Eigen::Vector3d w_; ///< Angular velocity [rad/sec] typically expressed in IMU body frame
  ImuCalibration* imuCalibration_ = nullptr; ///< Structure holding all of the calibration parameters for the Imu residuals
  Eigen::Vector2d* gravity_rot_ = nullptr; ///< The roll/pitch orientation of the gravity vector optimized. This is only used when the 'optGravity' option is true in the ImuChain.
};

} // namespace confusion

#endif //CONFUSION_IMUMEAS_H
