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

#ifndef CONFUSION_IMU_CHAIN_H_
#define CONFUSION_IMU_CHAIN_H_

#include "ceres/ceres.h"
#include "confusion/utilities/distances.h"
#include "confusion/utilities/rotation_utils.h"
#include "confusion/utilities/ceres_utils.h"
#include <deque>
#include <Eigen/Core>

#include "confusion/utilities/imu_utils.h"
#include "confusion/ProcessChain.h"

namespace confusion {

//todo Note that we assume that the error weighting is constant local to the current parameter values! Should investigate this further...
//A complicated structure is required here to support different type ceres cost
//functions depending on whether the gravity direction attitude is being estimated
//while using a common cost function evaluator function.

class ImuChain;

bool evaluateImuChainCost(ImuChain *imuChain, double const *const *x,
                          double *residuals, double **jacobians);

class ImuChainCostFuntion : public ceres::SizedCostFunction<15, 3, 4, 3, 3, 3, 3, 4, 3, 3, 3> {
 public:
  ImuChainCostFuntion(ImuChain *imuChain) : imuChain_(imuChain) {}

  /**
   * Compute the error. This also computes the analytical Jacobians. See the Ceres
   * documentation for AnalyticalCostFunction for implementation details. NOTE:
   * The cost function assumes that the cost weighting (i.e. the stiffness matrix)
   * is constant. This seems to be a sufficiently good approximation.
   * @param parameters
   * @param residuals
   * @param jacobians
   * @return
   */
  bool Evaluate(double const *const *parameters,
                double *residuals,
                double **jacobians) const {
    return evaluateImuChainCost(imuChain_, parameters, residuals, jacobians);
  }

  ImuChain *imuChain_;
};

class ImuChainCostFuntionOptGravity : public ceres::CostFunction {
 public:
  ImuChainCostFuntionOptGravity(ImuChain *imuChain) : imuChain_(imuChain) {
    //We have to use a dynamically sized cost function here because we require 11 parameters
    //Set the parameter block sizes
    //[p0,q0,v0,p1,q1,v1,ba,bg,gr]
    std::vector<int> *param_sizes = mutable_parameter_block_sizes();
    param_sizes->push_back(3); //p0
    param_sizes->push_back(4); //q0
    param_sizes->push_back(3); //v0
    param_sizes->push_back(3); //ba0
    param_sizes->push_back(3); //bg0
    param_sizes->push_back(3); //p1
    param_sizes->push_back(4); //q1
    param_sizes->push_back(3); //v1
    param_sizes->push_back(3); //ba1
    param_sizes->push_back(3); //bg1
    param_sizes->push_back(2); //gravity_roll_pitch

    //Set the number of residuals
    set_num_residuals(15);
  }

  /**
   * Compute the error. This also computes the analytical Jacobians. See the Ceres
   * documentation for AnalyticalCostFunction for implementation details. NOTE:
   * The cost function assumes that the cost weighting (i.e. the stiffness matrix)
   * is constant. This seems to be a sufficiently good approximation.
   * @param parameters
   * @param residuals
   * @param jacobians
   * @return
   */
  bool Evaluate(double const *const *parameters,
                double *residuals,
                double **jacobians) const {
    return evaluateImuChainCost(imuChain_, parameters, residuals, jacobians);
  }

  ImuChain *imuChain_;
};

/**
 * This class is an error term for a chain of IMU measurements between two state instances.
 * The state is: [w_pos_i, w_quat_i, w_linVel_i, accelBias, gyroBias].
 * It assumes that the IMU state appears
 * at the front of the state vector. If this is not the case, a derived version
 * can be created with a derived initialize() function.
 */
class ImuChain : public ProcessChain {
 public:
  /**
   * Constructor
   * @param optGravity Set true when the roll and pitch of the gravity vector are used as optimized parameters.
   * 		When false, the world reference frame must be aligned with gravity.
   * @param startingStateParamIndex The state parameter indices linked to this
   * 		measurement model start at the specified index. It is then assumed that
   * 		the IMU translation, rotation, linear velocity, accel bias, and gyro
   * 		bias are ordered sequentially in this order starting from that index.
   */
  ImuChain(bool optGravity = true, int startingStateParamIndex = 0) :
      ProcessChain("IMU", false), optGravity_(optGravity),
      startingStateParamIndex_(startingStateParamIndex) {}

  ~ImuChain() {}

  bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                          std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                          std::vector<size_t> &stateParameterIndexVector,
                          std::vector<double *> &staticParameterDataVector) {
    auto firstMeasPtr = std::dynamic_pointer_cast<ImuMeas>(measurements_.front());

    stateParameterIndexVector.push_back(0);
    stateParameterIndexVector.push_back(1);
    stateParameterIndexVector.push_back(2);
    stateParameterIndexVector.push_back(3);
    stateParameterIndexVector.push_back(4);

    if (optGravity_)
      staticParameterDataVector.push_back(firstMeasPtr->gravity_rot_->data());

    if (firstMeasPtr->imuCalibration_->imu_use_loss_func) {
      std::unique_ptr<ceres::LossFunction>
          lossFunctionPtr_(new ceres::CauchyLoss(firstMeasPtr->imuCalibration_->imu_loss_coeff));
      lossFunctionPtr = std::move(lossFunctionPtr_);
    } else
      lossFunctionPtr.reset();

    if (optGravity_) {
      std::unique_ptr<ceres::CostFunction> constFunctionPtr_(new ImuChainCostFuntionOptGravity(this));
      costFunctionPtr = std::move(constFunctionPtr_);
    } else {
      std::unique_ptr<ceres::CostFunction> constFunctionPtr_(new ImuChainCostFuntion(this));
      costFunctionPtr = std::move(constFunctionPtr_);
    }

    return true;
  }

  int residualDimension() { return 15; }

//protected:
  const bool optGravity_;
  const int startingStateParamIndex_;
};

#include "confusion/models/impl/ImuChain.h"

} // namespace confusion

#endif // CONFUSION_IMU_CHAIN_H_
