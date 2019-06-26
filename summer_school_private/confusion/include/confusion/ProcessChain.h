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

#ifndef INCLUDE_CONFUSION_PROCESSCHAIN_H_
#define INCLUDE_CONFUSION_PROCESSCHAIN_H_

#include <memory>
#include <deque>
#include <string>
#include "confusion/Parameter.h"
#include "confusion/StaticParameterVector.h"
#include "confusion/ProcessMeasurement.h"

namespace confusion {

/**
 * The base class for a chain of process measurements linking two subsequent state instances.
 */
class ProcessChain {
  friend class State;
  friend class ConFusor;
  friend class BatchFusor;
  friend class Diagram;
 public:
  /**
   * Construct and empty ProcessChain
   *
   * @param name Name of the associated measurement type
   * @param dynamicParameters When true, the cost function will be built for
   * 			each optimization problem. If false, the cost function is built
   * 			once and then used over subsequent fusion problems.
   */
  ProcessChain(std::string name, bool dynamicParameters) :
      name_(name), dynamicParameters_(dynamicParameters) {}

  virtual ~ProcessChain() {}

  /**
   * @brief Create the cost function for this ProcessChain and establish the links
   * between the cost function and the state and static parameters.
   *
   * Create the cost function for this ProcessChain and establish the links
   * between the cost function and the state and static parameters. If
   * dynamicParameters_ is true, this will be called during the creation of
   * every optimization problem, otherwise it will be called once and the same
   * cost function and linkings will be used until destruction. Note that if
   * dynamicParameters_ is false, it is assumed that a dynamic ceres cost function
   * type is not used, unless more than 10 parameters are used in the cost function.
   *
   * @param costFunctionPtr This should be populated with the created cost function
   * @param lossFunctionPtr This should be populated with the created loss function. Set this as null when no loss function should be used.
   * @param stateParameterIndexVector A vector of the state parameter indices which are linked to this cost function (the same set of parameters are linked on the starting and ending side of the chain)
   * @param staticParameterDataVector A vector of the static parameter data addresses linked to this cost function
   * @return Return true when the cost function was successfully created and can be used in the sensor fusion problem
   */
  virtual bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                                  std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                                  std::vector<size_t> &stateParameterIndexVector,
                                  std::vector<double *> &staticParameterDataVector) = 0;

  /**
   * @brief Assign the times of the leading and trailing states
   *
   * This is called by the MeasurementManager once a complete set of process measurements
   * between two state instances has been received.
   *
   * @param tStart The time of the state at the front end of the process chain.
   * @param tEnd The time of the state at the back end of the process chain.
   */
  void assignTimes(double tStart, double tEnd) {
    tStart_ = tStart;
    tEnd_ = tEnd;
    ready_ = true;
  }

  /**
   * The dimension of the residual computed by the associated cost function.
   */
  virtual int residualDimension() = 0;

  /**
   * Sanity checks for the process chain. Return false if a problem is detected.
   */
  virtual bool check() {
    bool res = true;

    if (initialized_) {
      if (linkedStartStateParameters_.empty()) {
        std::cout << "ERROR: Process chain does not have any state parameters assigned to the starting state!"
                  << std::endl;
        res = false;
      }
      if (linkedEndStateParameters_.empty()) {
        std::cout << "ERROR: Process chain does not have any state parameters assigned to the trailing state!"
                  << std::endl;
        res = false;
      }

      //Check for process measurement at or before the leading state
      if (!measurements_.empty()) { //We allow there to be no associated process measurements to support purely random walk process models
        if (measurements_.front()->t() > tStart()) {
          std::cout << "WARNING: There is no process measurement at or before the leading state at t=" << tStart() <<
                    "; t_process_front=" << measurements_.front()->t() << std::endl;
        }
        if (measurements_.size() > 1 && measurements_[1]->t() <= tStart_) {
          std::cout << "ERROR: There are too many process measurements at or before the leading state at t=" << tStart()
                    << std::endl;
          res = false;
        }

        //Check for process measurements coming at or after the following state
        if (measurements_.back()->t() >= tEnd()) {
          std::cout << "ERROR: ProcessChain has a measurement following the trailing state! "
                       "t_state=" << tEnd() << ". t_meas_back=" << measurements_.back()->t() << std::endl;
          res = false;
        }
      }

      if (linkedStartStateParameters_.size() + linkedEndStateParameters_.size() +
          linkedStaticParameters_.size() != parameterDataVector_.size()) {
        std::cout << "ERROR in ProcessChain! Mismatch between "
                     " the number of linked parameters and parameter blocks!" << std::endl;
      }
    }

    return res;
  }

  //todo Make these protected? They are accessed by at least State and StateVector
  std::deque<std::shared_ptr<ProcessMeasurement>>
      measurements_; ///< The process measurements that make up the process chain

  const double tStart() const { return tStart_; }
  const double tEnd() const { return tEnd_; }
  const bool initialized() const { return initialized_; }
  const bool ready() const { return ready_; }
  const std::string name() const { return name_; }
  ceres::ResidualBlockId residualBlockId() { return residualBlockId_; }

  void enable() { enable_ = true; }
  void disable() {
    enable_ = false;
    initialized_ = false;
  }
  const bool isEnabled() const { return enable_; }

//todo Making this public for WacoPropagator testing
//protected:
  /**
   * @brief Add the associated cost function to the optimization problem. This
   * is called every time the MHE problem is built and solved.
   *
   * @param problem Pointer to the underlying Ceres problem
   * @param startStateParameters The vector of parameters associated with the leading state
   * @param endStateParameters The vector of parameters associated with the trailing state
   * @param staticParameterVector The vector of static parameters
   * @return Return true when the cost function was successfully added to the problem, false otherwise
   */
  bool addCostToProblem(ceres::Problem *problem,
                        std::vector<Parameter> &startStateParameters,
                        std::vector<Parameter> &endStateParameters,
                        StaticParameterVector &staticParameterVector) {
    if (!enable_)
      return false;
    else if (dynamicParameters_ || !initialized_) {
      costFunctionPtr_.reset();
      lossFunctionPtr_.reset();
      parameterDataVector_.clear();
      linkedStartStateParameters_.clear();
      linkedEndStateParameters_.clear();
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
                     " are no state or static parameters linked to the ProcessChain cost function " << name_ << "!" << std::endl;
        return false;
      }

      // Parse the linked parameters specified and build the objects required for linking in ConFusion
      // Parameters must always be ordered as [leading state parameters, trailing state parameters, static parameters]
      for (auto &index: stateParameterIndexVector) {
        if (index >= startStateParameters.size() || index < 0) {
          std::cout << "ERROR: Specified state parameter index of " << index
                    << " out of range in ProcessChain::addCostToProblem "
                       "for measurement type " << name_ << std::endl;
          abort();
        }
        linkedStartStateParameters_.push_back(&startStateParameters[index]);
        parameterDataVector_.push_back(startStateParameters[index].data_);
      }
      for (auto &index: stateParameterIndexVector) {
        if (index >= endStateParameters.size() || index < 0) {
          std::cout << "ERROR: Specified state parameter index of " << index
                    << " out of range in ProcessChain::addCostToProblem "
                       "for measurement type " << name_ << std::endl;
          abort();
        }
        linkedEndStateParameters_.push_back(&endStateParameters[index]);
        parameterDataVector_.push_back(endStateParameters[index].data_);
      }
      for (auto &dataPtr: staticParameterDataVector) {
        Parameter *parameter = staticParameterVector.getParameter(dataPtr);
        if (!parameter) {
          std::cout << "ERROR: Couldn't find a linked static parameter in ProcessChain::addCostToProblem "
                       "for measurement type " << name_ << std::endl;
          abort();
        }
        linkedStaticParameters_.push_back(parameter);
        parameterDataVector_.push_back(dataPtr);
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
    for (auto &param: linkedStartStateParameters_)
      param->active_ = true;
    for (auto &param: linkedEndStateParameters_)
      param->active_ = true;
    for (auto &param: linkedStaticParameters_)
      param->active_ = true;

    return true;
  }

 protected:

  /**
   * @brief Reset the process chain. This is used when a state is reset after stopping tracking.
   */
  void reset() {
    measurements_.clear();
    initialized_ = false;
    linkedStartStateParameters_.clear();
    linkedEndStateParameters_.clear();
    linkedStaticParameters_.clear();
    costFunctionPtr_.reset();
    lossFunctionPtr_.reset();
    parameterDataVector_.clear();

    derivedReset();
  }

  /**
   * This should reset any internal memory in the derived ProcessChain when reset() is called.
   */
  virtual void derivedReset() {};

  std::vector<Parameter *> linkedStartStateParameters_; ///< The parameters of the leading state which are linked to this ProcessChain
  std::vector<Parameter *> linkedEndStateParameters_; ///< The parameters of the trailing state which are linked to this ProcessChain
  std::vector<Parameter *> linkedStaticParameters_; ///< The static parameters which are linked to this ProcessChain

  std::unique_ptr<ceres::CostFunction> costFunctionPtr_; ///< Pointer to the underlying cost function. The measurement classes hold ownership to the cost function so that it can be reused over multiple sensor fusion problems.
  std::unique_ptr<ceres::LossFunction> lossFunctionPtr_; ///< Pointer to the associated loss function. The measurement classes hold ownership to the cost function so that it can be reused over multiple sensor fusion problems. When no loss function is used, set to null.
  std::vector<double *> parameterDataVector_; ///< The vector of all parameter block addresses linked to the cost function. This is what is sent to ceres when the cost function is added to the ceres optimization problem.

 private:
  double tStart_ = 0.0; ///< The timestamp of the state at the front end of the process chain.
  double tEnd_ = 0.0; ///< The timestamp of the state at the back end of the process chain.
  std::string name_; ///< Name assigned to the measurement type. This is only used for diagnostics and for the automatically generated MHE diagrams.
  bool dynamicParameters_; ///< If true, the parameters assigned to the class can be changed after the cost function is build the first time.
  bool initialized_ = false; ///< Indicates if the associated cost function is active in the current optimization problem.
  bool ready_ = false; ///< This is set true once all of the ProcessMeasurements have been added and the start and end times have been assigned.
  bool enable_ = true; ///< If false, this measurement will be ignored and not considered when building the sensor fusion problems.
  ceres::ResidualBlockId residualBlockId_; ///< The address of the residual block tied to this measurement. This is used to marginalize states without rebuilding the problem.
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_PROCESSCHAIN_H_ */
