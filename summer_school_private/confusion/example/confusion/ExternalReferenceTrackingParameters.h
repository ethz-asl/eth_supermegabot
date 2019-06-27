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

//todo Delete this! Not needed for ERT!

#ifndef INCLUDE_CONFUSION_EXAMPLES_ERTPARAMETERS_H_
#define INCLUDE_CONFUSION_EXAMPLES_ERTPARAMETERS_H_

#include <Eigen/Core>

struct ExternalReferenceTrackingParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  void initialize() {
    imuCalibration_.cov_imu_nominal_.setIdentity();
    imuCalibration_.cov_imu_nominal_.block<3, 3>(0, 0) *= wi_stddev_ * wi_stddev_;
    imuCalibration_.cov_imu_nominal_.block<3, 3>(3, 3) *= ai_stddev_ * ai_stddev_;
    imuCalibration_.cov_imu_nominal_.block<3, 3>(6, 6) *= bg_stddev_ * bg_stddev_;
    imuCalibration_.cov_imu_nominal_.block<3, 3>(9, 9) *= ba_stddev_ * ba_stddev_;

    imuCalibration_.g_w_ << 0.0, 0.0, imuCalibration_.gravityMagnitude_;

    initialized_ = true;
  }

  bool initialized_ = false;

  double wi_stddev_;
  double ai_stddev_;
  double bg_stddev_;
  double ba_stddev_;

  double twi_init_stddev;    //[m]
  double qwi_init_stddev;    //[rad]
  double vwi_init_stddev;    //[m/s]
  double ba_init_stddev;     //[m/s2]
  double bg_init_stddev;     //[rad/s]
                             //	double ftr_init_stddev; //[rad]
  double t_c_i_init_stddev;  //[m]
  double q_c_i_init_stddev;  //[rad]

  confusion::ImuCalibration imuCalibration_;
};

#endif /* INCLUDE_CONFUSION_EXAMPLES_ERTPARAMETERS_H_ */
