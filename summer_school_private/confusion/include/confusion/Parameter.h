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

#ifndef INCLUDE_CONFUSION_PARAMETERBASE_H_
#define INCLUDE_CONFUSION_PARAMETERBASE_H_

#include <vector>
#include <string>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include "confusion/LocalParameterization.h"
#include "confusion/utilities/StaticParameterRandomWalkProcess.h"

namespace confusion {
/**
 * Class containing all of the internal status information of estimated parameters (either
 * as part of a state or static parameter).
 */
class Parameter {
	friend class FixedParameter;
	friend class PriorConstraint;
	friend class ConFusor;
public:
	/**
	 * Create a new Parameter. Parameters are set with constant_ = false by default.
	 * @param data Pointer to the start of the array of parameter data. The Parameter does NOT take ownership of the data.
	 * @param size Global size of the parameter
	 * @param name Name of the parameter (just used for diagnostics and in the Diagram class)
	 * @param constant If true, the parameter value will not be optimized
	 * @param parameterization Pointer to an associated local parameterization. Set to nullptr when no local parameterization is used.
	 */
	Parameter(double* data, size_t size, const std::string &name, bool constant = false,
			std::shared_ptr<LocalParameterizationBase> parameterization = nullptr):
				data_(data), parameterization_(parameterization),
				size_(size), name_(name), constant_(constant) {
		if (parameterization_)
			localSize_ = parameterization_->LocalSize();
		else
			localSize_ = size_;
	}

	/**
	 * Create a new Parameter. With this constructor, constant_ = false by default.
	 * @param data Pointer to the start of the array of parameter data. The Parameter does NOT take ownership of the data.
	 * @param size Global size of the parameter
	 * @param name Name of the parameter (just used for diagnostics and in the Diagram class)
	 * @param parameterization Pointer to an associated local parameterization. Set to nullptr when no local parameterization is used.
	 */
	Parameter(double* data, size_t size, const std::string &name,
			std::shared_ptr<LocalParameterizationBase> parameterization):
				data_(data), parameterization_(parameterization),
				size_(size), name_(name), constant_(false) {
		if (parameterization_)
			localSize_ = parameterization_->LocalSize();
		else
			localSize_ = size_;
	}

	/**
	 * When there is a prior likelihood on an estimated parameter, this function
	 * sets the confidence in the initial parameter values.
	 * @param w The stiffness matrix (inverse standard deviation) of the prior confidence in the parameter values. Should be a square matrix with size equal to the local size of the parameter.
	 */
	void setInitialConstraintWeighting(const Eigen::MatrixXd& w) {
		if (w.rows() != int(localSize_) || w.cols() != int(localSize_)) {
			//todo Throw an exception
			std::cout << "ERROR: Trying to set an initial constraint weighting with the wrong size!" << std::endl;
			return;
		}
		if (active_) {
			std::cout << "ERROR: Trying to set an initial constraint weighting for a parameter that is already active! "
					"This should be done before starting fusion" << std::endl;
			return;
		}
		initialConstraintWeighting_ = w;
		immediatelyAddToPrior_ = true;
	}

	/**
	 * Add the parameter to the current optimization problem being built
	 * @param problem Pointer to the ceres optimization problem
	 */
	void addToProblem(ceres::Problem* problem) {
		if (constant_) {
			//todo Remove this check once I am sure that its not needed
			if (problem->HasParameterBlock(data_))
				problem->SetParameterBlockConstant(data_);
			else
				std::cout << "Trying to set a parameter const that isn't in the ceres problem???" << std::endl;
		}
		else if (parameterization_) {
//			std::cout << "Setting local parameterization for variable of size " << size_ << std::endl;
			problem->SetParameterization(data_, parameterization_.get());
		}

		if (randomWalkProcessActive_) {
			if (constant_) {
				//todo Remove this check once I am sure that its not needed
				if (problem->HasParameterBlock(rwpPriorSideParameterValue_.data()))
					problem->SetParameterBlockConstant(rwpPriorSideParameterValue_.data());
				else
					std::cout << "Trying to set a rwp parameter const that isn't in the ceres problem???" << std::endl;
			}
			else if (parameterization_) {
//			std::cout << "Setting local parameterization for variable of size " << size_ << std::endl;
				problem->SetParameterization(rwpPriorSideParameterValue_.data(), parameterization_.get());
			}
		}
	}

	//
	/**
	 * Clone the parameter, copying the current data values into the new parameter, but
	 * maintaining the current parameter's internal state. This is used when a
	 * new state is created from the previous state during online operation.
	 * @param data The address of the new parameter data
	 * @return The cloned Parameter
	 */
	Parameter clone(double* data) const {
		Parameter param(data, size_, name_, constant_, parameterization_);

		//Copy over the values of the parameter
		for (size_t i=0; i<size_; ++i)
			data[i] = data_[i];

		//Copy over the internal state
		param.immediatelyAddToPrior_ = immediatelyAddToPrior_;
		param.initialConstraintWeighting_ = initialConstraintWeighting_;

		return param;
	}

	/**
	 * Attach a random walk process to this static parameter. It will add a process constraint between the prior
	 * constraint and the static parameter. That means that two parameters will then be optimized for this static
	 * parameter; one between the prior and the process, and one between the process and the other residuals in the MHE.
	 * @param processNoise  Process white noise. Units [unit of parameter * sqrt(sec)].
	 * @return Shared pointer to the random walk process cost function.
	 */
	void attachRandomWalkProcess(const double &processNoise) {
		if (randomWalkProcess_) {
			std::cout << "WARNING: User requested to attach a random walk process to a static parameter with one "
						 "already attached. Ignoring the request." << std::endl;
			return;
		}

		//Copy the parameter value over for the prior side parameter
		rwpPriorSideParameterValue_.resize(size());
		for (size_t i=0; i<size(); ++i)
			rwpPriorSideParameterValue_(i) = data_[i];

		//Create the RWP
		if (parameterization_)
			randomWalkProcess_ = std::make_shared<StaticParameterRandomWalkProcess>(
					rwpPriorSideParameterValue_.data(), data_, parameterization_.get(), processNoise);
		else
			randomWalkProcess_ = std::make_shared<StaticParameterRandomWalkProcess>(
					rwpPriorSideParameterValue_.data(), data_, size(), processNoise);
	}

	/**
	 * Detach the random walk process from this static parameter. Note that the rwp cost function will be destroyed, but
	 * may stay active in the MHE problem last built. It will not be included in the next problem built.
	 */
	void detachRandomWalkProcess() {
		if (!randomWalkProcess_) {
			std::cout << "WARNING: User requested to detach a random walk process from a static parameter without one "
						 "attached. Ignoring the request." << std::endl;
			return;
		}

		randomWalkProcess_ = nullptr;
		randomWalkProcessActive_ = false;
	}

	/**
	 * Convenience function to print the current data values
	 */
	void toCout() const {
		std::cout << "[";
		for(size_t i=0; i<size_; ++i) {
			std::cout << data_[i];
			if (i+1 < size_)
				std::cout << ",";
		}
		std::cout << "]";
	}

	/**
	 * Set the parameter constant. This should only be used before sensor fusion
	 * has started! If you want to set a parameter constant during operation
	 * use ConFusor::setStaticParamsConstant. Setting a state parameter constant
	 * during operation is not yet supported.
	 */
	void setConstant() {
		if (priorConstraintActive_) {
			//todo Note that it is not yet supported to set parameters of the
			//first state constant during operation
			std::cout << "ERROR: Cannot directly set parameter " << name_ <<
					" constant because it is active in the prior constraint! "
					"To set a parameter constant during operation, use "
					"ConFusor::setStaticParamsConstant" << std::endl;
			return;
		}
		constant_ = true;
		randomWalkProcessActive_ = false;
	}
	/**
	 * Set the parameter to be optimized, starting from the creation of the next
	 * optimization problem.
	 */
	void unsetConstant() { constant_ = false; }

	const size_t size() const { return size_; }
	const size_t localSize() const { return localSize_; }
	const bool isConstant() const { return constant_; }
	const bool isActive() const { return active_; }
	const bool immediatelyAddToPrior() const { return immediatelyAddToPrior_; }
	const bool priorConstraintActive() const { return priorConstraintActive_; }
	const int priorConstraintLocalPosition() const { return priorConstraintLocalPosition_; }
	const std::string name() const { return name_; }

	const bool isRandomWalkProcessAttached() const { return randomWalkProcess_.get(); }
	const bool isRandomWalkProcessActive() const { return randomWalkProcessActive_; }
	double* rwpPriorSideParameterAddress() { return rwpPriorSideParameterValue_.data(); }

	double* data_; ///< Address of the associated parameter data. The Parameter does not take ownership of the data.
	std::shared_ptr<LocalParameterizationBase> parameterization_ = nullptr; ///< Pointer to the parameter's local parameterization. Set to nullptr when no local parameterization is active.
	bool active_ = false; ///< This flag indicates if the parameter is linked to any of the cost functions in the current optimization problem.
							//todo It needs to be set by many different classes, so it is left public for now

protected:
	/**
	 * Update the RWP, copying the new value of the optimized parameter to the prior side parameter and setting the
	 * new dt. This is called after one or more states are marginalized in the MHE. Called by ConFusion::PriorConstraint.
	 * @param dt The delta time from the time of the last marginalized state to that of the new marginalized state.
	 */
	void updateRandomWalkProcess(const double &dt) {
		if (!randomWalkProcess_) {
			std::cout << "ERROR: Parameter::updateRandomWalkProcess called with no RWP attached!" << std::endl;
			abort();
		}

		//Update the value of the prior side parameter
		for (size_t i=0; i<size(); ++i)
			rwpPriorSideParameterValue_(i) = data_[i];

		//Update the dt in the cost function
		randomWalkProcess_->updateDt(dt);

		randomWalkProcessActive_ = true;
	}

	/**
	 * Add the random walk process to the MHE problem. Called by ConFusion::PriorConstraint.
	 * @param problem Pointer to the current MHE problem
	 */
	void addRandomWalkProcessToProblem(ceres::Problem* problem) {
		if (!randomWalkProcess_) {
			std::cout << "ERROR: Parameter::addRandomWalkProcessToProblem called with no RWP attached!" << std::endl;
			abort();
		}

		if (!randomWalkProcess_) {
			std::cout << "ERROR: No RWP attached when Parameter::addRandomWalkProcessesToProblem was called??" << std::endl;
			abort();
		}

		randomWalkProcess_->addCostFunctionToProblem(problem);
	}

	size_t size_; ///< Global size of the parameter (e.g. 4 for a quaternion)
	size_t localSize_; ///< Local size of the parameter (e.g. 3 for a quaternion)
	std::string name_; ///< Name of the parameter. Just used for diagnostics.
	bool constant_ = false; ///< When true, the parameter will not be optimized

	bool immediatelyAddToPrior_ = false; ///< When true, a prior distribution on this parameter is specified, and the parameter will be immediately added to the prior constraint when it first goes active
	Eigen::MatrixXd initialConstraintWeighting_; ///< The inverse standard deviation for the prior confidence in the parameter values
	bool priorConstraintActive_ = false; ///< Indicates if this parameter is linked to the prior constraint of the current problem
	int priorConstraintLocalPosition_; ///< Indicates where the parameter sits among the static parameters (NOT including the state parameters!) in the prior constraint.

    std::shared_ptr<StaticParameterRandomWalkProcess> randomWalkProcess_ = nullptr;
    bool randomWalkProcessActive_ = false;
	Eigen::VectorXd rwpPriorSideParameterValue_; ///< When a RWP is attached to this parameter, this copy of it is constrained by the prior and the rwp.
};

//This class copies and fixes the values of a parameter for use in marginalization
class FixedParameter {
	friend class PriorCost;
public:
	FixedParameter(Parameter paramIn): size_(paramIn.size_),
			localSize_(paramIn.localSize_),	parameterization_(paramIn.parameterization_),
			constant_(paramIn.constant_), name_(paramIn.name_) {
		copyOfData_.resize(paramIn.size_);
		for (size_t i=0; i<paramIn.size_; ++i) {
		  //If a RWP is active, we want to copy the value of the prior side parameter
			if (paramIn.isRandomWalkProcessActive())
				copyOfData_(i) = paramIn.rwpPriorSideParameterValue_(i);
			else
				copyOfData_(i) = paramIn.data_[i];
		}
	}

	const size_t size() const { return size_; }
	const size_t localSize() const { return localSize_; }
	const bool isConstant() const { return constant_; }
	const std::string name() const { return name_; }
	const Eigen::VectorXd data() const { return copyOfData_; }

protected:
	Eigen::VectorXd copyOfData_;
	size_t size_; //The size of the parameter before applying the local parameterization (e.g. 4 for a quaternion)
	size_t localSize_;
	std::shared_ptr<LocalParameterizationBase> parameterization_;
	bool constant_;
	std::string name_;
};

} //namespace confusion

#endif /* INCLUDE_CONFUSION_PARAMETERBASE_H_ */
