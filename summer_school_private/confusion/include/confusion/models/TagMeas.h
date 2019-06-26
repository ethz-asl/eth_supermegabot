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

#ifndef INCLUDE_CONFUSION_TAGMEAS_H_
#define INCLUDE_CONFUSION_TAGMEAS_H_

#include <Eigen/Core>
#include <array>
#include "confusion/UpdateMeasurement.h"
#include "confusion/utilities/Pose.h"
#include "confusion/utilities/ceres_utils.h"
#include "confusion/modules/apriltag/apriltag_utils.h"

namespace confusion {

struct TagMeasCalibration {
  Eigen::Matrix<double, 3, 4> projMat_;
  std::array<Eigen::Vector3d, 4> t_t_corner_;
  double w_cx_;
  double w_cy_;
  bool tagUseLossFunction_ = false;
  double tagLossCoefficient_ = 0.0;
  double tagSize_;
  double ftr_init_stddev_;
};

class TagMeas;

class TagCost {
 public:
  TagCost(TagMeas *tagMeas) : tagMeas_(tagMeas) {}

  template<typename T>
  bool operator()(T const *t_w_i_, T const *q_w_i_,
                  T const *t_c_i_, T const *q_c_i_,
                  T const *t_w_t_, T const *q_w_t_, T *residual_) const;

  TagMeas *tagMeas_;
};

class TagMeas : public UpdateMeasurement {
 public:
  TagMeas(double t, const std::string referenceFrameName, confusion::Pose<double> &T_c_i,
          const std::array<std::array<double, 2>, 4> corners,
          const TagMeasCalibration &tagMeasCalibration, int measurementType = 0, int startingStateParamIndex = 0) :
      UpdateMeasurement(measurementType, t, "Tag", false),
      referenceFrameName_(referenceFrameName), T_c_i_(T_c_i), corners_(corners),
      tagMeasCalibration_(tagMeasCalibration), T_w_ref_ptr_(nullptr),
      startingStateParamIndex_(startingStateParamIndex) {}

  void assignExternalReferenceFrame(std::shared_ptr<confusion::Pose<double>> &T_w_tag) {
    if (T_w_ref_ptr_) {
      std::cout << "ERROR: Trying to assign an externalReferenceFrame to a TagMeas that already has one assigned to it!"
                << std::endl;
      return;
    }
    T_w_ref_ptr_ = T_w_tag;
  }

  ~TagMeas() {}

  //Used for state initialization
  Pose<double> getTct() {
//    std::cout << "[getTct]: fx=" << tagMeasCalibration_.projMat_(0, 0) << ", fy=" << tagMeasCalibration_.projMat_(1, 1) << ", cx=" << tagMeasCalibration_.projMat_(0, 2) <<", cy=" << tagMeasCalibration_.projMat_(1, 2) << ", tagSize=" << tagMeasCalibration_.tagSize_ << std::endl;
//    for (int i=0; i<4; ++i)
//      std::cout << "Corner: " << corners_[i][0] << "," << corners_[i][1] << std::endl;
    Eigen::Matrix4d T_c_t_mat = getRelativeTransform(corners_,
                                                     tagMeasCalibration_.tagSize_,
                                                     tagMeasCalibration_.projMat_(0, 0),
                                                     tagMeasCalibration_.projMat_(1, 1),
                                                     tagMeasCalibration_.projMat_(0, 2),
                                                     tagMeasCalibration_.projMat_(1, 2));
    return Pose<double>(T_c_t_mat);
  }

  bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                          std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                          std::vector<size_t> &stateParameterIndexVector,
                          std::vector<double *> &staticParameterDataVector) {
    stateParameterIndexVector.push_back(0);
    stateParameterIndexVector.push_back(1);
    staticParameterDataVector.push_back(T_c_i_.trans.data());
    staticParameterDataVector.push_back(T_c_i_.rot.coeffs().data());
    staticParameterDataVector.push_back(T_w_ref_ptr_->trans.data());
    staticParameterDataVector.push_back(T_w_ref_ptr_->rot.coeffs().data());

    std::unique_ptr<ceres::CostFunction>
        constFunctionPtr_(new ceres::AutoDiffCostFunction<TagCost, 8, 3, 4, 3, 4, 3, 4>(new TagCost(this)));
    costFunctionPtr = std::move(constFunctionPtr_);

    if (!tagMeasCalibration_.tagUseLossFunction_)
      lossFunctionPtr.reset();
    else {
      std::unique_ptr<ceres::LossFunction>
          lossFunctionPtr_(new ceres::HuberLoss(tagMeasCalibration_.tagLossCoefficient_));
      lossFunctionPtr = std::move(lossFunctionPtr_);
    }

    return true;
  }

  int residualDimension() { return 8; }

  bool checkDerived() {
    bool res = true;
    if (!T_w_ref_ptr_) {
      std::cout << "ERROR: TagMeas at " << t() << " of type " << measType()
                << " doesn't have an externalReferenceFrame assigned to it!" << std::endl;
      res = false;
    }
    return res;
  }

  const confusion::Pose<double> &T_c_i() const { return T_c_i_; }

  const std::string referenceFrameName_;
  confusion::Pose<double> &T_c_i_;
  const std::array<std::array<double, 2>, 4>
      corners_; //4x2 array of [x,y] image coordinates ordered BOTTOM_LEFT, BOTTOM_RIGHT, TOP_RIGHT, TOP_LEFT
  const TagMeasCalibration &tagMeasCalibration_;
//	ceres::CostFunction* pose_cf = nullptr;

  std::shared_ptr<confusion::Pose<double>> T_w_ref_ptr_;

  const int
      startingStateParamIndex_; ///< Indicates the index of the state parameters which contains the position of the measured body pose. It is assumed that the orientation is then the next parameter.
};

template<typename T>
bool TagCost::operator()(T const *t_w_i_, T const *q_w_i_,
                         T const *t_c_i_, T const *q_c_i_,
                         T const *t_w_t_, T const *q_w_t_, T *residual_) const {
#ifdef COST_DEBUG
	std::cout << "Start tag cost computation for t=" << tagMeas_->t() << std::endl;
#endif
  Eigen::Matrix<T, 3, 1> t_w_i(t_w_i_);
  Eigen::Quaternion<T> q_w_i = Eigen::Map<const Eigen::Quaternion<T>>(q_w_i_);
  Pose<T> T_w_i(t_w_i, q_w_i);

  Eigen::Matrix<T, 3, 1> t_c_i(t_c_i_);
  Eigen::Quaternion<T> q_c_i = Eigen::Map<const Eigen::Quaternion<T>>(q_c_i_);
  Pose<T> T_c_i(t_c_i, q_c_i);

  Eigen::Matrix<T, 3, 1> t_w_t(t_w_t_);
  Eigen::Quaternion<T> q_w_t = Eigen::Map<const Eigen::Quaternion<T>>(q_w_t_);
  Pose<T> T_w_t(t_w_t, q_w_t);

  //Get the estimated tag pose in the camera frame
  Pose<T> T_c_t_est = T_c_i * T_w_i.inverse() * T_w_t;

  for (int i = 0; i < 4; ++i) {
    //Get the position of each tag corner in the camera frame
    Eigen::Matrix<T, 3, 1> t_c_corner = T_c_t_est * tagMeas_->tagMeasCalibration_.t_t_corner_[i].cast<T>();

    //Project the corner into the image plane
    T px_x_est = T(tagMeas_->tagMeasCalibration_.projMat_(0, 0)) * (t_c_corner(0) / t_c_corner(2))
        + T(tagMeas_->tagMeasCalibration_.projMat_(0, 2));
    T px_y_est = T(tagMeas_->tagMeasCalibration_.projMat_(1, 1)) * (t_c_corner(1) / t_c_corner(2))
        + T(tagMeas_->tagMeasCalibration_.projMat_(1, 2));

    //Compute the residual
    residual_[2 * i] = (T(tagMeas_->corners_[i][0]) - px_x_est) * T(tagMeas_->tagMeasCalibration_.w_cx_);
    residual_[2 * i + 1] = (T(tagMeas_->corners_[i][1]) - px_y_est) * T(tagMeas_->tagMeasCalibration_.w_cy_);

//std::cout << "x meas: " << tagMeas_->corners_[i][0] << ", x est: "<< px_x_est << "; y meas: " << tagMeas_->corners_[i][1] << " y est: "<< px_y_est << std::endl;
  }

#ifdef COST_DEBUG
  Eigen::Matrix<double,8,1> res;
  getDoubles(residual_,8,res.data());
  std::cout << tagMeas_->name() << " cost: " << res.transpose() << std::endl;
#endif
  return true;
}

} // namespace confusion

#endif /* INCLUDE_CONFUSION_TAGMEAS_H_ */
