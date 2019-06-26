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

#ifndef INCLUDE_CONFUSION_POSEMEAS_H
#define INCLUDE_CONFUSION_POSEMEAS_H

#include "confusion/UpdateMeasurement.h"
#include "confusion/utilities/Pose.h"

namespace confusion {

struct PoseMeasConfig {
  double w_trans;                         //[m] Translational inverse standard deviation
  double w_rot;                           //[rad] Rotational inverse standard deviation
  bool useLossFunction = false;           // Use a Huber loss function for the residual
  double lossCoefficient = 0.0;           // Huber loss coefficient
  double sensorOffset_trans_init_stddev;  //[m] Initial certainty in the translational part of the extrinsic calibration
  double sensorOffset_rot_init_stddev;    //[m] Initial certainty in the rotational part of the extrinsic calibration
  bool optTranslationalExtrinsic = false; //Optimize the translational part of the extrinsic calibration online
  bool optRotationalExtrinsic = false;    //Optimize the rotational part of the extrinsic calibration online
  bool optScale = false;                  //Optimize scale online
};

class PoseMeas;

/**
 * Computes an error between the estimated target pose (extracted from the state) and the measured
 * target pose (measured with some arbitrary external tracking system).
 * We measure T_wa_ta.
 * We estimate T_w_i.
 * w: World frame of the estimator
 * wa: World frame of the external tracking system
 * i: Frame of the IMU which is tracked by the estimator
 * ta: Traget frame of the external tracking system. This frame must be ridigly coupled to the IMU
 * frame.
 * We can additionally estimate the two offsets to the external measurements:
 * T_w_wa: World frame offset. If gravity aligned, you could use a
 * confusion::FixedYawParameterization on this offset's orientation with GRAVITY_OPT off
 * T_i_ba: Target frame offset
 */
class PoseCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PoseCost(PoseMeas* pose_meas) : pose_meas_(pose_meas) {}
  ~PoseCost(){};

  template <typename T>
  bool operator()(T const* t_w_i_, T const* q_w_i_, T const* t_w_wa_, T const* q_w_wa_,
                  T const* t_i_ba_, T const* q_i_ba_, T* residual_) const;

  template <typename T>
  bool operator()(T const* t_w_i_, T const* q_w_i_, T const* t_w_wa_, T const* q_w_wa_,
                  T const* t_i_ba_, T const* q_i_ba_, T const* scale_, T* residual_) const;

 private:
  PoseMeas* pose_meas_;
};

/**
 * Measurement from an external tracking system
 */
class PoseMeas : public confusion::UpdateMeasurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PoseMeas(const double t, const std::string referenceFrameName,
           confusion::Pose<double>& T_imu_sensor, const confusion::Pose<double>& T_wa_ba,
           const PoseMeasConfig& config, const int measurement_type = 0,
           int starting_state_param_index = 0)
      : confusion::UpdateMeasurement(measurement_type, t, "Pose", false),
        referenceFrameName_(referenceFrameName),
        T_imu_sensor_(T_imu_sensor),
        T_wa_ba_(T_wa_ba),
        scale_(NULL),
        config_(config),
        starting_state_param_index_(starting_state_param_index) {}

  PoseMeas(const double t, const std::string referenceFrameName,
           confusion::Pose<double>& T_imu_sensor, const confusion::Pose<double>& T_wa_ba,
           const PoseMeasConfig& config, double* scale, const int measurement_type = 0,
           int starting_state_param_index = 0)
      : confusion::UpdateMeasurement(measurement_type, t, "Pose", false),
        referenceFrameName_(referenceFrameName),
        T_imu_sensor_(T_imu_sensor),
        T_wa_ba_(T_wa_ba),
        scale_(scale),
        optimize_scale_(true),
        config_(config),
        starting_state_param_index_(starting_state_param_index) {}

  void assignExternalReferenceFrame(std::shared_ptr<confusion::Pose<double>> erf) {
    if (T_w_ref_ptr_) {
      std::cout << "ERROR: Trying to assign an externalReferenceFrame to a PoseMeas that already "
                   "has one assigned to it!"
                << std::endl;
      return;
    }
    T_w_ref_ptr_ = erf;
  }

  ~PoseMeas() {}

  bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                          std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                          std::vector<size_t> &stateParameterIndexVector,
                          std::vector<double *> &staticParameterDataVector) {
    stateParameterIndexVector.push_back(0);
    stateParameterIndexVector.push_back(1);

    staticParameterDataVector.push_back(T_w_ref_ptr_->trans.data());
    staticParameterDataVector.push_back(T_w_ref_ptr_->rot.coeffs().data());
    staticParameterDataVector.push_back(T_imu_sensor_.trans.data());
    staticParameterDataVector.push_back(T_imu_sensor_.rot.coeffs().data());

    std::unique_ptr<ceres::CostFunction> costFunctionPtr_(
        new ceres::AutoDiffCostFunction<PoseCost, 6, 3, 4, 3, 4, 3, 4>(new PoseCost(this)));
    costFunctionPtr = std::move(costFunctionPtr_);

    if (!config_.useLossFunction)
      lossFunctionPtr.reset();
    else {
      std::unique_ptr<ceres::LossFunction> lossFunctionPtr_(
          new ceres::HuberLoss(config_.lossCoefficient));
      lossFunctionPtr = std::move(lossFunctionPtr_);
    }

    return true;
  }

  confusion::Pose<double> getMeasuredImuPose() {
    if (!T_w_ref_ptr_) {
      std::cout << "ERROR: PoseMeas::getMeasuredImuPose called with invalid T_w_ref_ptr_!"
                << std::endl;
      return confusion::Pose<double>();
    }
    return (*T_w_ref_ptr_) * T_wa_ba_ * T_imu_sensor_.inverse();
  }

  int residualDimension() { return 6; }

  const confusion::Pose<double>& T_wa_ba() const { return T_wa_ba_; }
  const confusion::Pose<double>& T_imu_sensor() const { return T_imu_sensor_; }
  const PoseMeasConfig& config() const { return config_; }
  const std::string& referenceFrameName() const { return referenceFrameName_; }

 private:
  // Name of the reference frame. Used to link the measurement to the correct external frame offset.
  const std::string referenceFrameName_;
  // Measured pose
  const confusion::Pose<double> T_wa_ba_;
  // Target frame offset to IMU (only used to set the address of the parameters internally)
  confusion::Pose<double>& T_imu_sensor_;
  // Scale applied on translation. Usually 1.
  double* scale_;
  bool optimize_scale_ = false;
  // Calibrations
  const PoseMeasConfig& config_;

  std::shared_ptr<confusion::Pose<double>> T_w_ref_ptr_;

  // Indicates the index of the state parameters which contains the position of the measured body
  // pose.
  // The orientation must then be the next parameter.
  const int starting_state_param_index_;
};

template <typename T>
bool PoseCost::operator()(T const* t_w_i_, T const* q_w_i_, T const* t_w_wa_, T const* q_w_wa_,
                          T const* t_i_ba_, T const* q_i_ba_, T* residual_) const {
#ifdef COST_DEBUG
  std::cout << "Starting pose cost computation" << std::endl;
#endif

  Eigen::Matrix<T, 3, 1> t_w_i(t_w_i_);
  Eigen::Quaternion<T> q_w_i(q_w_i_);
  confusion::Pose<T> T_w_i(t_w_i, q_w_i);

  Eigen::Matrix<T, 3, 1> t_w_wa(t_w_wa_);
  Eigen::Quaternion<T> q_w_wa(q_w_wa_);
  confusion::Pose<T> T_w_wa(t_w_wa, q_w_wa);

  Eigen::Matrix<T, 3, 1> t_i_ba(t_i_ba_);
  Eigen::Quaternion<T> q_i_ba(q_i_ba_);
  confusion::Pose<T> T_i_ba(t_i_ba, q_i_ba);

  // Get the estimated measurement
  confusion::Pose<T> T_wa_ba_est = T_w_wa.inverse() * T_w_i * T_i_ba;
  // T_wa_ba_est.print("T_wa_ba_est");
  // pose_meas_->T_wa_ba().print("T_wa_ba");

  // Compute the residuals
  confusion::VectorDistance(pose_meas_->T_wa_ba().trans.data(), T_wa_ba_est.trans.data(),
                            residual_);
  confusion::QuatDistance(pose_meas_->T_wa_ba().rot, T_wa_ba_est.rot, residual_ + 3);

  residual_[0] *= T(pose_meas_->config().w_trans);
  residual_[1] *= T(pose_meas_->config().w_trans);
  residual_[2] *= T(pose_meas_->config().w_trans);
  residual_[3] *= T(pose_meas_->config().w_rot);
  residual_[4] *= T(pose_meas_->config().w_rot);
  residual_[5] *= T(pose_meas_->config().w_rot);

#ifdef COST_DEBUG
  std::cout << "Done PoseCost" << std::endl;

  Eigen::Matrix<double, 6, 1> res;
  confusion::getDoubles(residual_, 6, res.data());
  std::cout << "Pose cost = [" << res.transpose() << "]" << std::endl;
#endif

  return true;
}

template <typename T>
bool PoseCost::operator()(T const* t_w_i_, T const* q_w_i_, T const* t_w_wa_, T const* q_w_wa_,
                          T const* t_i_ba_, T const* q_i_ba_, T const* scale_, T* residual_) const {
#ifdef COST_DEBUG
  std::cout << "Starting pose cost computation" << std::endl;
#endif

  Eigen::Matrix<T, 3, 1> t_w_i(t_w_i_);
  Eigen::Quaternion<T> q_w_i(q_w_i_);
  confusion::Pose<T> T_w_i(t_w_i, q_w_i);

  Eigen::Matrix<T, 3, 1> t_w_wa(t_w_wa_);
  Eigen::Quaternion<T> q_w_wa(q_w_wa_);
  confusion::Pose<T> T_w_wa(t_w_wa, q_w_wa);

  Eigen::Matrix<T, 3, 1> t_i_ba(t_i_ba_);
  Eigen::Quaternion<T> q_i_ba(q_i_ba_);
  confusion::Pose<T> T_i_ba(t_i_ba, q_i_ba);

  // Get the estimated measurement
  confusion::Pose<T> T_wa_ba_est = T_w_wa.inverse() * T_w_i * T_i_ba;
  // T_wa_ba_est.print("T_wa_ba_est");
  // pose_meas_->T_wa_ba().print("T_wa_ba");

  // Compute the residuals
  confusion::ScaledVectorDistance(pose_meas_->T_wa_ba().trans.data(), T_wa_ba_est.trans.data(),
                                  scale_, residual_);
  confusion::QuatDistance(pose_meas_->T_wa_ba().rot, T_wa_ba_est.rot, residual_ + 3);

  residual_[0] *= T(pose_meas_->config().w_trans);
  residual_[1] *= T(pose_meas_->config().w_trans);
  residual_[2] *= T(pose_meas_->config().w_trans);
  residual_[3] *= T(pose_meas_->config().w_rot);
  residual_[4] *= T(pose_meas_->config().w_rot);
  residual_[5] *= T(pose_meas_->config().w_rot);

#ifdef COST_DEBUG
  std::cout << "Done PoseCost" << std::endl;

  Eigen::Matrix<double, 6, 1> res;
  confusion::getDoubles(residual_, 6, res.data());
  std::cout << "Pose cost = [" << res.transpose() << "]" << std::endl;
#endif

  return true;
}

}  // namespace confusion

#endif  // INCLUDE_CONFUSION_POSEMEAS_H
