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

#ifndef INCLUDE_CONFUSION_STATE_H_
#define INCLUDE_CONFUSION_STATE_H_

#include <memory>
#include <deque>
#include <vector>
#include <Eigen/Core>
#include "confusion/ProcessChain.h"
#include "confusion/UpdateMeasurement.h"
#include "confusion/Parameter.h"
#include "confusion/PriorConstraint.h"
#include "confusion/StaticParameterVector.h"


namespace confusion {


/**
 * @brief The base class for the estimated state.
 */
class State {
	friend class StateVector;
	friend class MeasurementManager;
	friend class ConFusor;
	friend class BatchFusor;
	friend class Diagram;
	friend class Logger;
public:

	State(double t, int numProcessSensors, int numUpdateSensors):
		t_(t), updateMeasurements_(numUpdateSensors), processChains_(numProcessSensors),
    numProcessSensors_(numProcessSensors), numUpdateSensors_(numUpdateSensors) { }

	virtual ~State() { }

	/**
	 * \brief Set the confidence in the initial values of the first state parameters.
	 *
	 * Set the prior confidence in the initial values of the first state parameters.
	 * These weightings, which are the inverse standard deviation of the paremeters,
	 * define the initial ConFusor::priorConstraint_ once each state parameter
	 * becomes active. If no initial state weighting is set, there will be no
	 * prior constraint on the first state and the parameters will only be
	 * constrained by the linked measurements. It can be useful to constrain
	 * some of the state parameters which are not immediately observable from the
	 * first measurements (e.g. IMU biases).
	 *
	 * @param stateParamWeightings A vector of square matrices which represent
	 * the confidence in the initial values of the state parameters. The length
	 * of the vector should match the number of State::parameters_. Each matrix
	 * should have size matching the local size of the parameter at the same index.
	 * To only set an initial constraint for some parameters, pass a matrix of size
	 * 0x0 for the parameters which should not have an initial constraint.
	 */
	void setInitialStateWeighting(std::vector<Eigen::MatrixXd> stateParamWeightings) {
		if (stateParamWeightings.size() != parameters_.size()) {
			std::cout << "ERROR: setInitialStateConfidence called with the wrong number of weighting matrices. "
					"If a state parameter has no prior constraint, send an empty matrix for it. "
					"# of matrices: " << stateParamWeightings.size() <<
					", expected #: " << parameters_.size() << std::endl;
		}

		for (size_t i=0; i<stateParamWeightings.size(); ++i) {
			if (stateParamWeightings[i].rows() > 0)
				parameters_[i].setInitialConstraintWeighting(stateParamWeightings[i]);
		}
	}

	/**
	 * A general interface to print a state's parameters.
	 */
	virtual void print() { }

	const double t() const { return t_; }
	double t_; ///< Timestamp assigned to the state. todo Should be private but sometimes its nice to update this, e.g. after ImuPropagation

    const size_t numProcessSensors() const { return numProcessSensors_; }
    const size_t numUpdateSensors() const { return numUpdateSensors_; }

	std::vector<Parameter> parameters_; ///< Holds all of the parameter blocks in the state

	//todo Provide const getters so that these can be protected
	std::vector<std::deque<std::shared_ptr<UpdateMeasurement>>> updateMeasurements_; ///< Holds all of the update measurements linked to the state.
	std::vector<std::shared_ptr<ProcessChain>> processChains_; ///< Holds all of the process chains starting from this state.

protected:
	/**
	 * @brief Add an update measurement to the state.
	 *
	 * This is called by MeasurementManager. The user can optionally define a
	 * derived implementation to create and initialize new static parameters
	 * from the measurement (e.g. to initialize new features from an image).
	 *
	 * @param measPtr Pointer to the measurement being added.
	 * @param staticParameters The static parameters being optimized.
	 * 		These are passed so that new static parameters can be added from the
	 * 		new update measurement.
	 * @return Return true if the measurement was accepted and added to the state, false otherwise.
	 */
	virtual bool addUpdateMeasDerived(std::shared_ptr<UpdateMeasurement> measPtr,
			StaticParameterVector& staticParameters) {
		//This check is in the default derived implementation so that the user
		//can optionally pass measurements beyond the specified number of sensors
		if (measPtr->measType() < int(updateMeasurements_.size()))
			updateMeasurements_[measPtr->measType()].push_back(measPtr);
		else {
			std::cout << "WARNING: State tried adding an update measurement with undefined type! Dropping the measurement." << std::endl;
			return false;
		}

		return true;
	}

	/**
	 * @brief Add a process measurement to the state.
	 *
	 * This is called by MeasurementManager. The user can optionally define a
	 * derived implementation to provide custom behavior.
	 *
	 * @param measPtr Pointer to the measurement being added.
	 */
	virtual void addProcessMeas(std::shared_ptr<ProcessMeasurement> measPtr) {
		//todo Is there a case where a derived implementation is required?
		//todo Return a boolean like addUpdateMeas?
		if (measPtr->measType() < int(processChains_.size()))
			processChains_[measPtr->measType()]->measurements_.push_back(measPtr);
		else
			std::cout << "WARNING: State tried adding a process measurement with undefined type! Dropping the measurement." << std::endl;
//std::cout << "Added process meas at " << measPtr->t() << " to state at " << t_ << ". There are now " << processChains_[measPtr->measType_]->measurements_.size() << " measurements_." << std::endl;
	}

	/**
	 * @brief Initialize the first state when sensor fusion starts.
	 *
	 * A ConFusor is seeded with an uninitialized version of the first derived state.
	 * This function is called by the MeasurementManager to initialize that first
	 * state to start fusion. The user-defined implementation can decide which
	 * measurements should be used for initialization, and therefore what the time
	 * of the first state will be. If the measurements required to initialize the state
	 * have not yet been received, false should be returned.
	 *
	 * @param processMeasBuffer The process measurements received, which can be
	 * 			used to initialize the state.
	 * @param updateMeasBuffer The update measurements received, which can be
	 * 			used to initialize the state.
	 * @param staticParameters The static parameters in the sensor fusion problem.
	 * 			New static parameters can be added when initializing the first
	 * 			state or the values of static parameters can be initialized here
	 * 			as well.
	 * @return Return true if the first state was successfully initialized. This
	 * 			will signal to the ConFusor that sensor fusion can begin. Return
	 * 			false if the first state has not yet been initialized.
	 */
	virtual bool initFirstState(
			const std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>>& processMeasBuffer,
			const std::vector<std::deque<std::shared_ptr<UpdateMeasurement>>>& updateMeasBuffer,
			StaticParameterVector& staticParameters) = 0;

	/**
	 * @brief Create the next state from the preceding state and the measurements
	 * 			received so far.
	 *
	 * This is called by the MeasurementManager to create new states during
	 * sensor fusion. The user implementation should use the passed measurements
	 * to initialize the state parameters. Once created, the new state will be
	 * optimized within the MHE. If the measurements required to initialize the
	 * next state have not yet been received, return a null pointer.
	 *
	 * @param processMeasBuffer The process measurements received, which can be
	 * 			used to initialize the state.
	 * @param updateMeasBuffer The update measurements received, which can be
	 * 			used to initialize the state.
	 * @param staticParameters The static parameters in the sensor fusion problem.
	 * 			New static parameters can be added when initializing the
	 * 			state or the values of static parameters can be initialized here
	 * 			as well.
	 * @return Return a pointer to the new state. If a new state could not be created,
	 * return a null pointer.
	 */
	virtual std::shared_ptr<State> createNextState(
			const std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>>& processMeasBuffer,
			const std::vector<std::deque<std::shared_ptr<UpdateMeasurement>>>& updateMeasBuffer,
			StaticParameterVector& staticParameters) = 0;

	/**
	 * Create a new state from a single update measurement. This is called in attempt to create new state between two
	 * existing states when an asynchonous update measurement is receive back in time (e.g. when there are two update
	 * sensors with different latencies). The process measurements already assigned to the state are used for
	 * initialization as well.
	 * @return Return a pointer to the new state. If a new state could not be created,
	 * return a null pointer.
	 */
  std::shared_ptr<State> createNextState(
      const std::shared_ptr<UpdateMeasurement>& updateMeas,
      StaticParameterVector& staticParameters) {
    //Create dummy measurement buffers
    std::vector<std::deque<std::shared_ptr<UpdateMeasurement>>> updateMeasBuffer(numUpdateSensors_);
      updateMeasBuffer[updateMeas->measType_].push_back(updateMeas);
    std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>> processMeasBuffer;
    for (size_t i=0; i<numProcessSensors_; ++i) {
      processMeasBuffer.push_back(processChains_[i]->measurements_);
    }

    return createNextState(processMeasBuffer, updateMeasBuffer, staticParameters);
  }



	const size_t numProcessSensors_; ///< The number of process measurement sensors/models in the sensor fusion problem.
	const size_t numUpdateSensors_; ///< The number of update measurement sensors/models in the sensor fusion problem.

private:
	/**
	 * Get the dimension of all of the update residuals linked to the state and
	 * the process chain residuals starting from the state.
	 *
	 * @return The dimension of the residuals linked to the state.
	 */
	size_t residualDimension() {
		size_t errorDim = 0;
		for (size_t i=0; i<numUpdateSensors_; ++i)
			for (size_t j=0; j<updateMeasurements_[i].size();  ++j)
				errorDim += updateMeasurements_[i][j]->residualDimension();
		for (size_t i=0; i<numProcessSensors_; ++i)
			errorDim += processChains_[i]->residualDimension();
		return errorDim;
	}

	/**
	 * Make sure all of the static parameters linked to the state are added to the
	 * prior constraint before marginalizing out the state. This is called internally
	 * by the ConFusor for maginalization.
	 */
	void addNewParametersToPrior(StaticParameterVector& staticParameters, PriorConstraint& priorConstraint) {
		for (size_t i=0; i<numUpdateSensors_; ++i) {
			for (size_t j=0; j<updateMeasurements_[i].size(); ++j) {
			  if (updateMeasurements_[i][j]->isEnabled()) {
                for (size_t k = 0; k < updateMeasurements_[i][j]->linkedStaticParameters_.size(); ++k) {
                  priorConstraint.addStaticParameter(*updateMeasurements_[i][j]->linkedStaticParameters_[k]);
                }
              }
			}
		}
		for (size_t i=0; i<numProcessSensors_; ++i) {
			for (size_t j=0; j<processChains_[i]->linkedStaticParameters_.size(); ++j) {
				priorConstraint.addStaticParameter(*processChains_[i]->linkedStaticParameters_[j]);
			}
		}
	}

	/**
	 * Some sanity checks that all of the required class members have been
	 * properly set up.
	 */
	void check() {
		if (parameters_.empty())
			std::cout << "ERROR: State parameters_ is empty! You must specify the state parameters in the derived class!" << std::endl;
		if (processChains_.size() != numProcessSensors_) {
			std::cout << "ERROR: The processChains_ vector is not properly setup!" << std::endl;
		}
		for (const auto& processChain: processChains_) {
			if (!processChain)
				std::cout << "ERROR: One process chain is NULL in processChains_!" << std::endl;
		}
	}

	/**
	 * Reset the state. This is only used when tracking is stopped and restarted.
	 */
	void reset() {
		//Clear out the update measurements
		for (size_t i=0; i<numUpdateSensors_; ++i)
			updateMeasurements_[i].clear();
		//Reset the process chains
		for (size_t i=0; i<numProcessSensors_; ++i)
			processChains_[i]->reset();
	}

	/**
	 * @brief Assign a new update measurement to the state.
	 *
	 * Assign a new update measurement to the state. This is called by the MeasurementManager.
	 *
	 * @param measPtr Pointer to the measurement being added.
	 * @param staticParameters The static parameters being optimized.
	 * 		These are passed so that new static parameters can be added from the
	 * 		new update measurement (e.g. if a new feature is observed in the latest image).
	 * @return Return true if the measurement was accepted and added to the state, false otherwise. If the measurement is not accepted, it will be dropped.
	 */
	bool addUpdateMeas(std::shared_ptr<UpdateMeasurement> measPtr,
			StaticParameterVector& staticParameters) {
		return addUpdateMeasDerived(measPtr, staticParameters);
	}
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_STATE_H_ */
