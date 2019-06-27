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

#ifndef INCLUDE_CONFUSION_UPDATEMEASUREMENT_H_
#define INCLUDE_CONFUSION_UPDATEMEASUREMENT_H_

#include <memory>
#include <deque>
#include <string>
#include <ceres/ceres.h>
#include "confusion/Parameter.h"
#include "confusion/StaticParameterVector.h"

namespace confusion {

/**
 * The base class for update measurements (i.e. measurements which provide information
 * about the value of the parameters of a single state instance)
 */
class UpdateMeasurement {

  friend class State;
  friend class ConFusor;
  friend class BatchFusor;
  friend class Diagram;

 public:
  /**
   * Base class constructor
   *
   * @param measType The integer identifier for the measurement type.
   * @param t Timestamp for the measurement
   * @param name A name for this type of measurement
   * @param dynamicParameters When true, the cost function will be built for
   * 			each optimization problem. If false, the cost function is built
   * 			once and then used over subsequent fusion problems.
   */
  UpdateMeasurement(int measType, double t, std::string name, bool dynamicParameters) :
      measType_(measType), t_(t), name_(name), dynamicParameters_(dynamicParameters) {}

  virtual ~UpdateMeasurement() {}

  /**
   * @brief Create the cost function for this UpdateMeasurement and establish the links
   * between the cost function and the state and static parameters.
   *
   * Create the cost function for this UpdateMeasurement and establish the links
   * between the cost function and the state and static parameters. If
   * dynamicParameters_ is true, this will be called during the creation of
   * every optimization problem, otherwise it will be called once and the same
   * cost function and linkings will be used until destruction. Note that if
   * dynamicParameters_ is false, it is assumed that a dynamic ceres cost function
   * type is not used, unless more than 10 parameters are used in the cost function.
   *
   * @param costFunctionPtr This should be populated with the created cost function
   * @param lossFunctionPtr This should be populated with the created loss function. Set this as null when no loss function is used.
   * @param stateParameterIndexVector A vector of the state parameter indices which are linked to this cost function
   * @param staticParameterDataVector A vector of the static parameter data addresses linked to this cost function
   * @return Return true when the cost function was successfully created and can be used in the sensor fusion problem
   */
  virtual bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                                  std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                                  std::vector<size_t> &stateParameterIndexVector,
                                  std::vector<double *> &staticParameterDataVector) = 0;

  /**
   * The derived update measurement class must specify its associated residual size.
   */
  virtual int residualDimension() = 0;

  /**
   * Perform sanity checks on the measurement setup
   *
   * @return Return false if a problem is identified
   */
  bool check() {
    bool res = true;

    if (initialized_) {
      if (linkedStateParameters_.empty()) {
        std::cout << "ERROR: Update measurement at t=" << t_ <<
                  " does not have any state parameters assigned to it!" << std::endl;
        res = false;
      }
      if (linkedStateParameters_.size() + linkedStaticParameters_.size() != parameterDataVector_.size()) {
        std::cout << "ERROR in UpdateMeasurement type " << measType() << "! Mismatch between "
                                                                         " the number of linked parameters and parameter blocks!"
                  << std::endl;
      }
    }

    if (!checkDerived())
      res = false;

    return res;
  }

  /**
   * Evaluate the underlying cost function for use outside of the sensor fusion problem.
   *
   * @return The computed residual vector
   */
  std::vector<double> evaluateResidual() {
    if (!initialized()) {
      std::cout << "The user requested to externally evaluate a cost function"
                   " that has not yet been initialized!" << std::endl;
      return std::vector<double>();
    }
    std::vector<double> residual(residualDimension());
    costFunctionPtr_->Evaluate(parameterDataVector_.data(), residual.data(), nullptr);
    return residual;
  }

  /**
   * User defined sanity checks. Return false if a problem is detected.
   */
  virtual bool checkDerived() { return true; }

  const int measType() const { return measType_; }
  const double &t() const { return t_; }
  const bool initialized() const { return initialized_; }
  const std::string name() const { return name_; }
  ceres::ResidualBlockId residualBlockId() { return residualBlockId_; }

  void enable() { enable_ = true; }

  virtual void disable() {
    enable_ = false;
    initialized_ = false;
  }
  const bool &isEnabled() const { return enable_; }

  /**
   * Tells the ConFusion::MeasurementManager that the measurement should not be used and assigned to a ConFusion::State.
   * Intended to be called from State::initFirstState or State::createNextState.
   */
  void dropMeasurement() { dropMeasurement_ = true; }
  const bool &shouldBeDropped() const { return dropMeasurement_; }

 protected:
  /**
  * @brief Add the associated cost function to the optimization problem. This
  * is called every time the MHE problem is built and solved.
  *
  * @param problem Pointer to the underlying Ceres problem
  * @param stateParameters The vector of parameters associated with the state
  * @param staticParameterVector The vector of static parameters
  * @return Return true when the cost function was successfully added to the problem, false otherwise
  */
  bool addCostToProblem(ceres::Problem *problem,
                        std::vector<Parameter> &stateParameters,
                        StaticParameterVector &staticParameterVector) {
    if (!enable_)
      return false;
    else if (dynamicParameters_ || !initialized_) {
      costFunctionPtr_.reset();
      lossFunctionPtr_.reset();
      parameterDataVector_.clear();
      linkedStateParameters_.clear();
      linkedStaticParameters_.clear();

      std::vector<size_t> stateParameterIndexVector;
      std::vector<double *> staticParameterDataVector;
      if (!createCostFunction(costFunctionPtr_, lossFunctionPtr_, stateParameterIndexVector, staticParameterDataVector))
        return false;

      if (!costFunctionPtr_) {
        std::cout << "ERROR: createCostFunction returned true but the"
                     " cost function was not created!" << std::endl;
        return false;
      }

      if (stateParameterIndexVector.empty() && staticParameterDataVector.empty()) {
        std::cout << "ERROR: createCostFunction returned true but there"
                     " are no state or static parameters linked to the UpdateMeasurment cost function type " << measType_ << "!" << std::endl;
        return false;
      }

      // Parse the linked parameters specified and build the objects required for linking in ConFusion
      // Parameters must always be ordered with state parameters first
      for (auto &index: stateParameterIndexVector) {
        if (index >= stateParameters.size() || index < 0) {
          std::cout << "ERROR: Specified state parameter index of " << index
                    << " out of range in UpdateMeasurement::addCostToProblem "
                       "for measurement type " << measType_ << std::endl;
          abort();
        }
        linkedStateParameters_.push_back(&stateParameters[index]);
        parameterDataVector_.push_back(stateParameters[index].data_);
      }
      for (auto &dataPtr: staticParameterDataVector) {
        Parameter *parameter = staticParameterVector.getParameter(dataPtr);
        if (!parameter) {
          std::cout << "ERROR: Couldn't find a linked static parameter in UpdateMeasurement::addCostToProblem "
                       "for measurement type " << measType_ << std::endl;
          abort();
        }
        linkedStaticParameters_.push_back(parameter);
        parameterDataVector_.push_back(dataPtr);
      }

      if (parameterDataVector_.empty()) {
        std::cout << "ERROR: createCostFunction returned true but the"
                     " parameter vector is empty!" << std::endl;
        return false;
      }

      initialized_ = true;
    }

    //Add the cost function to the problem
    //Note that we assume that a Dynamic ceres cost function type is not used unless there are more than 10 associated parameters
    int numParams = parameterDataVector_.size();
    if (dynamicParameters_ || numParams > 10)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(), lossFunctionPtr_.get(),
                                                   parameterDataVector_);
    else if (numParams == 10)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(),
                                                   lossFunctionPtr_.get(),
                                                   parameterDataVector_[0],
                                                   parameterDataVector_[1],
                                                   parameterDataVector_[2],
                                                   parameterDataVector_[3],
                                                   parameterDataVector_[4],
                                                   parameterDataVector_[5],
                                                   parameterDataVector_[6],
                                                   parameterDataVector_[7],
                                                   parameterDataVector_[8],
                                                   parameterDataVector_[9]);
    else if (numParams == 9)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(),
                                                   lossFunctionPtr_.get(),
                                                   parameterDataVector_[0],
                                                   parameterDataVector_[1],
                                                   parameterDataVector_[2],
                                                   parameterDataVector_[3],
                                                   parameterDataVector_[4],
                                                   parameterDataVector_[5],
                                                   parameterDataVector_[6],
                                                   parameterDataVector_[7],
                                                   parameterDataVector_[8]);
    else if (numParams == 8)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(),
                                                   lossFunctionPtr_.get(),
                                                   parameterDataVector_[0],
                                                   parameterDataVector_[1],
                                                   parameterDataVector_[2],
                                                   parameterDataVector_[3],
                                                   parameterDataVector_[4],
                                                   parameterDataVector_[5],
                                                   parameterDataVector_[6],
                                                   parameterDataVector_[7]);
    else if (numParams == 7)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(),
                                                   lossFunctionPtr_.get(),
                                                   parameterDataVector_[0],
                                                   parameterDataVector_[1],
                                                   parameterDataVector_[2],
                                                   parameterDataVector_[3],
                                                   parameterDataVector_[4],
                                                   parameterDataVector_[5],
                                                   parameterDataVector_[6]);
    else if (numParams == 6)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(),
                                                   lossFunctionPtr_.get(),
                                                   parameterDataVector_[0],
                                                   parameterDataVector_[1],
                                                   parameterDataVector_[2],
                                                   parameterDataVector_[3],
                                                   parameterDataVector_[4],
                                                   parameterDataVector_[5]);
    else if (numParams == 5)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(),
                                                   lossFunctionPtr_.get(),
                                                   parameterDataVector_[0],
                                                   parameterDataVector_[1],
                                                   parameterDataVector_[2],
                                                   parameterDataVector_[3],
                                                   parameterDataVector_[4]);
    else if (numParams == 4)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(),
                                                   lossFunctionPtr_.get(),
                                                   parameterDataVector_[0],
                                                   parameterDataVector_[1],
                                                   parameterDataVector_[2],
                                                   parameterDataVector_[3]);
    else if (numParams == 3)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(),
                                                   lossFunctionPtr_.get(),
                                                   parameterDataVector_[0],
                                                   parameterDataVector_[1],
                                                   parameterDataVector_[2]);
    else if (numParams == 2)
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(), lossFunctionPtr_.get(),
                                                   parameterDataVector_[0], parameterDataVector_[1]);
    else
      residualBlockId_ = problem->AddResidualBlock(costFunctionPtr_.get(), lossFunctionPtr_.get(),
                                                   parameterDataVector_[0]);

    //Active the linked state and static parameters
    for (auto &param: linkedStateParameters_)
      param->active_ = true;
    for (auto &param: linkedStaticParameters_)
      param->active_ = true;

    return true;
  }

  std::vector<Parameter *>
      linkedStateParameters_; ///< The parameters of the associated state which are linked to this measurement
  std::vector<Parameter *> linkedStaticParameters_; ///< The static parameters which are linked to this measurement

  std::unique_ptr<ceres::CostFunction>
      costFunctionPtr_; ///< Pointer to the underlying cost function. The measurement classes hold ownership to the cost function so that it can be reused over multiple sensor fusion problems.
  std::unique_ptr<ceres::LossFunction>
      lossFunctionPtr_; ///< Pointer to the associated loss function. The measurement classes hold ownership to the cost function so that it can be reused over multiple sensor fusion problems. When no loss function is used, set to null.
  std::vector<double *>
      parameterDataVector_; ///< The vector of all parameter block addresses linked to the cost function. This is what is sent to ceres when the cost function is added to the ceres optimization problem.

 protected:
  bool enable_ =
      true; ///< If false, this measurement will be ignored and not considered when building the sensor fusion problems.
  bool initialized_ =
      false; ///< Indicates if the associated cost function is active in the current optimization problem.
  bool dropMeasurement_ =
      false; ///< The user can set this to true in State::initFirstState or State::createNextState to tell ConFusion to drop the measurement before assigning it to a state. Has no effect once the measurement is assigned to a state.

 private:
  int measType_; ///< Measurement index identifier
  double t_; ///< Timestamp
  std::string
      name_; ///< Name assigned to the measurement type. This is only used for printing and for the automatically generated MHE diagrams
  bool
      dynamicParameters_; ///< If true, the parameters assigned to the class can be changed after the cost function is build the first time.

  ceres::ResidualBlockId
      residualBlockId_; ///< The address of the residual block tied to this measurement. This is used to marginalize states without rebuilding the problem.
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_UPDATEMEASUREMENT_H_ */
