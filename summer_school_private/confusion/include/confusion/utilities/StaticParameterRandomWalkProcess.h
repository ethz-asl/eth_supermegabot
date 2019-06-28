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

#ifndef INCLUDE_CONFUSION_MODELS_STATICPARAMETERRANDOMWALKPROCESS_H_
#define INCLUDE_CONFUSION_MODELS_STATICPARAMETERRANDOMWALKPROCESS_H_

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "confusion/LocalParameterization.h"

namespace confusion {

//todo Should make another version for N many state/static parameters with dynamic cost function size and inheriting from ProcessChain for general usage

/**
 * Random walk process chain, to apply to a single static parameter
 */
class StaticParameterRandomWalkProcess : protected ceres::CostFunction {
 public:
  StaticParameterRandomWalkProcess(double* priorParameterData, double* optParameterData,
                                   LocalParameterizationBase *parameterization, const double &processNoise) :
          parameterization_(parameterization),
          localSize_(parameterization->LocalSize()),
          globalSize_(parameterization->GlobalSize()),
          processNoise_(processNoise) {
    mutable_parameter_block_sizes()->push_back(globalSize_);
    mutable_parameter_block_sizes()->push_back(globalSize_);
    set_num_residuals(localSize_);

    parameterBlocks_.push_back(priorParameterData);
    parameterBlocks_.push_back(optParameterData);
  }

  StaticParameterRandomWalkProcess(double* priorParameterData, double* optParameterData,
                                   const size_t &globalSize, const double &processNoise) :
      parameterization_(nullptr),
      localSize_(globalSize),
      globalSize_(globalSize),
      processNoise_(processNoise) {
    mutable_parameter_block_sizes()->push_back(globalSize_);
    mutable_parameter_block_sizes()->push_back(globalSize_);
    set_num_residuals(globalSize_);

    parameterBlocks_.push_back(priorParameterData);
    parameterBlocks_.push_back(optParameterData);
  }

  void addCostFunctionToProblem(ceres::Problem* problem) {
    problem->AddResidualBlock(this, nullptr, parameterBlocks_);

    //Add the prior side local parameter as well. The parameterization for the other parameter is added externally.
    if (parameterization_)
      problem->SetParameterization(parameterBlocks_[0], parameterization_);
  }

  void updateDt(const double &dt) {
    w_ = 1.0 / processNoise_ / sqrt(dt);
//    std::cout << "w_rwp=" << w_ << std::endl;
  }

  //Parameter order is x_prior, x_opt
  bool Evaluate(double const* const* x, double* residuals, double** jacobians) const;

 protected:
  LocalParameterizationBase *parameterization_;
  const size_t localSize_; ///< Parameter local size
  const size_t globalSize_; ///< Parameter global size
  const double processNoise_; //< Process white noise. Units [unit of parameter / sqrt(sec)]
  double w_ = 0.0; ///< Process noise variance inverse-square-root w=1/sigma_x/sqrt(dt). Units [1 / unit of parameter/ sqrt(sec)].

  std::vector<double*> parameterBlocks_;
};

bool StaticParameterRandomWalkProcess::Evaluate(double const* const* x, double* residuals, double** jacobians) const {
#ifdef COST_DEBUG
  std::cout << "Starting StaticParameterRandomWalkProcessCost" << std::endl;
#endif
  if (parameterization_) {
    parameterization_->boxMinus(x[0], x[1], residuals);

    for (int i = 0; i < parameterization_->LocalSize(); ++i)
      residuals[i] *= w_;
  } else {
    for (size_t i=0; i<globalSize_; ++i)
      residuals[i] = w_ * (x[0][i] - x[1][i]);
  }
//  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > e_eigen(
//      residuals, localSize_, 1);
//  std::cout << "e_rwp for global size=" << globalSize_ << " : " << e_eigen.transpose() << std::endl;

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::MatrixXd dd_dx;
      if (parameterization_)
        dd_dx = parameterization_->boxMinusJacobianLeft(x[0], x[1]);
      else
        dd_dx.setIdentity(globalSize_, globalSize_);

      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > de_dx0(
          jacobians[0], localSize_, globalSize_);

      de_dx0 = w_ * dd_dx;
    }

    if (jacobians[1]) {
      Eigen::MatrixXd dd_dx;
      if (parameterization_)
        dd_dx = parameterization_->boxMinusJacobianRight(x[0], x[1]);
      else {
        dd_dx.setIdentity(globalSize_, globalSize_);
        dd_dx *= -1.0;
      }

      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > de_dx1(
          jacobians[1], localSize_, globalSize_);

      de_dx1 = w_ * dd_dx;
    }
  }

#ifdef COST_DEBUG
  std::cout << "StaticParameterRandomWalkProcessCost done" << std::endl;
#endif

  return true;
}

} //namespace confusion



#endif /* INCLUDE_CONFUSION_MODELS_STATICPARAMETERRANDOMWALKPROCESS_H_ */
