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

#ifndef INCLUDE_CONFUSION_BATCHFUSOR_H_
#define INCLUDE_CONFUSION_BATCHFUSOR_H_

//#define OPT_DE/BUG

#include <deque>
#include <memory>
#include <ceres/ceres.h>
//#include <ros/ros.h>
#include "confusion/StateVector.h"
#include "confusion/ProcessChain.h"
#include "confusion/UpdateMeasurement.h"
#include "confusion/MeasurementManager.h"
#include "confusion/StaticParameterVector.h"
#include "confusion/utilities/ceres_utils.h"


//todo Optionally specify a prior cost for the batch problem?

namespace confusion {

class BatchFusor {
public:
//	BatchFusor(std::shared_ptr<State> firstStatePtr): stateVector_(firstStatePtr) {
//		//Set the solver options
//		solverOptions_.minimizer_progress_to_stdout = true;
//		solverOptions_.max_num_iterations = 100;
//		solverOptions_.num_threads = 4;
//	}

	BatchFusor(StateVector& stateVector, StaticParameterVector& staticParameters):
		stateVector_(stateVector), staticParameters_(staticParameters) {
		//Set the solver options
		solverOptions_.minimizer_progress_to_stdout = true;
		solverOptions_.max_num_iterations = 100;
		solverOptions_.num_threads = 4;
	}

	~BatchFusor() {
		if (problem_)
			delete problem_;
	}



	void setSolverOptions(int numThreads, int maxIterations) {
		solverOptions_.max_num_iterations = maxIterations;
		solverOptions_.num_threads = numThreads;
	}

	void createStatesFromMeasurements(
		std::deque<std::shared_ptr<ProcessMeasurement>>& processMeasurements,
		std::deque<std::shared_ptr<UpdateMeasurement>>& updateMeasurements);

	void buildProblem();
	void optimize();

	void getResiduals(
		std::vector<Eigen::VectorXd>& processResiduals,
		std::vector<Eigen::VectorXd>& updateResiduals);

	void reset() {
		stateVector_.reset();
	}

	void setIterationCallback(std::shared_ptr<ceres::IterationCallback> callback) {
		callback_ = callback;
		solverOptions_.callbacks.push_back(callback_.get());
		solverOptions_.update_state_every_iteration = true;
	}

	void printParameterCovariance(double* d, std::string paramName) {
		Parameter* param = staticParameters_.getParameter(d);
		Eigen::MatrixXd cov;
		if (param)
			getCovariance(*problem_, param->data_, param->size(), param->data_, param->size(), paramName, &cov);
		std::cout << paramName << " cov:\n" << cov << std::endl;
	}

//	void setCallback();

	StateVector& stateVector_;
	StaticParameterVector& staticParameters_;

	ceres::Problem* problem_ = nullptr;
	ceres::Solver::Summary summary_;
	std::shared_ptr<ceres::IterationCallback> callback_;
	ceres::Solver::Options solverOptions_;
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_BATCHFUSOR_H_ */
