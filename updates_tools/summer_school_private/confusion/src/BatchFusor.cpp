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


#include "confusion/BatchFusor.h"

namespace confusion {

void BatchFusor::createStatesFromMeasurements(
		std::deque<std::shared_ptr<ProcessMeasurement>>& processMeasurements,
		std::deque<std::shared_ptr<UpdateMeasurement>>& updateMeasurements) {
	if (stateVector_.size() > 1) {
		std::cout << "ERROR in BatchFusor: You must call reset before createStatesFromMeasurements!" << std::endl;
		return;
	}

//	ros::Time t_start, t_end;
//	t_start = ros::Time::now();

	MeasurementManager measurementManager(
			stateVector_.front()->numProcessSensors_,
			stateVector_.front()->numUpdateSensors_);

	//Add all measurements to the measurement manager
	for (auto processMeas: processMeasurements)
		measurementManager.addProcessMeasurement(processMeas);
	for (auto updateMeas: updateMeasurements)
		measurementManager.addUpdateMeasurement(updateMeas);

	//Assign the measurements and build the state vector
	measurementManager.assignMeasurements(stateVector_, staticParameters_);
}

void BatchFusor::buildProblem() {
	stateVector_.check();
	staticParameters_.check();
//	stateVector_.print();

//	t_start = ros::Time::now();
	std::cout << "Building a batch problem consisting of " << stateVector_.states_.size() << " states and " <<
			staticParameters_.numParameters() << " static parameters" << std::endl;

	//Create the optimization problem and options
	//Note that the problem should not delete the cost functions and local parameterizations because we want to use them repeatedly as states move through the batch
	ceres::Problem::Options problemOptions;
	problemOptions.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problemOptions.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

	if(problem_)
		delete problem_;
	problem_ = new ceres::Problem(problemOptions);

	//Set up the batch problem
	//Start with all parameters deactivated and activate them as they get linked to measurements
	stateVector_.deactivateParameters();
	staticParameters_.deactivateParameters();

	for(int i=0; i<stateVector_.size(); ++i) {
		int numUpdateCosts = 0;
		int numProcessCosts = 0;
		for (int j=0; j<stateVector_[i]->numUpdateSensors_; ++j) {
			for (int k=0; k<stateVector_[i]->updateMeasurements_[j].size(); ++k) {
//				if (stateVector_[i]->updateMeasurements_[j][k]->initialized()) {
#ifdef OPT_DEBUG
					std::cout << "State at " << stateVector_[i]->t_ << ". Adding update cost type " << j << " at index " << k << std::endl;
#endif
					//todo Check nullptr even though it should never be null?
					if (stateVector_[i]->updateMeasurements_[j][k]->addCostToProblem(problem_,
							stateVector_[i]->parameters_, staticParameters_)) {
						++numUpdateCosts;
					}
//				}
			}
		}

		if (i < stateVector_.size()-1) { //The most recent state does not have any process measurements active
			for (int j=0; j<stateVector_[i]->numProcessSensors_; ++j) {
				if (stateVector_[i]->processChains_[j]->ready()) {
#ifdef OPT_DEBUG
					std::cout << "State at " << stateVector_[i]->t_ << ". Adding process cost type " << j << std::endl;
#endif
					if (stateVector_[i]->processChains_[j]->addCostToProblem(problem_,
							stateVector_[i]->parameters_, stateVector_[i+1]->parameters_,
							staticParameters_)) {
						++numProcessCosts;
					}
				}
			}
		}

#ifdef OPT_DEBUG
		std::cout << "Added " << numUpdateCosts << " update costs and " <<
				numProcessCosts << " process costs for the state at " << stateVector_[i]->t_ << std::endl;
#endif
	}

	//Add the active state parameters to the problem
	for(int i=0; i<stateVector_.size(); ++i) {
		int numParameters = 0;
		for (int j=0; j<stateVector_[i]->parameters_.size(); ++j) {
			if (stateVector_[i]->parameters_[j].isActive()) {
//std::cout << "i=" << i << "; j=" << j << std::endl;
				stateVector_[i]->parameters_[j].addToProblem(problem_);
				++numParameters;
			}
		}
#ifdef OPT_DEBUG
		std::cout << "Added " << numParameters << " of " << stateVector_[i]->parameters_.size() <<
				" parameters for the state at " << stateVector_[i]->t_ << std::endl;
#endif
	}

	//Add the active static parameters to the problem
	for (auto& iter: staticParameters_) {
		if (iter.second.isActive()) {
			iter.second.addToProblem(problem_);

#ifdef OPT_DEBUG
			std::cout << "Added static parameter with size " <<
					iter.second.size() << " to the problem. priorConstraintActive=" <<
					iter.second.priorConstraintActive() << std::endl;
#endif
		}
	}
}

void BatchFusor::optimize() {
	if (!problem_)
		std::cout << "ERROR: There is no problem to solve. Did you call buildProblem before optimize?" << std::endl;
	//Solve
	Solve(solverOptions_, problem_, &summary_);
//	t_end = ros::Time::now();

//	std::cout << summary.FullReport() << "\n";
}

void BatchFusor::getResiduals( //Eigen::VectorXd& priorResidual,
		std::vector<Eigen::VectorXd>& processResiduals,
		std::vector<Eigen::VectorXd>& updateResiduals) {
	if (!problem_) {
		std::cerr << "ERROR: There no previous problem available for residual "
				"computation! You need to optimize before calling getResiduals." << std::endl;
		return;
	}

	ceres::Problem::EvaluateOptions evalOptions;
	evalOptions.num_threads = solverOptions_.num_threads;

	double cost;
	std::vector<double> residuals;
	problem_->Evaluate(evalOptions, &cost, &residuals, nullptr, nullptr);

//	std::cout << "getResiduals computed cost=" << cost << " from " << residuals.size() << " residuals" << std::endl;

	Eigen::Map<Eigen::VectorXd> residuals_(residuals.data(), residuals.size());
//std::cout << "residuals: " << residuals_.transpose() << std::endl;

	//todo Be careful. Here the order of the residual blocks is assumed to be
	//consistent with how they are added to the problem in StateVector::addCostsToProblem
	int residual_index = 0;
	for (int i=0; i<stateVector_.size(); ++i) {
		for (int j=0; j<stateVector_[i]->numUpdateSensors_; ++j) {
			for (int k=0; k<stateVector_[i]->updateMeasurements_[j].size(); ++k) {
				Eigen::VectorXd updateResidual(stateVector_[i]->updateMeasurements_[j][k]->residualDimension()+2);
				updateResidual(0) = stateVector_[i]->t_;
				updateResidual(1) = (double)j;
				for (int l=0; l<stateVector_[i]->updateMeasurements_[j][k]->residualDimension(); ++l) {
					updateResidual(l+2) = residuals_[residual_index];
					++residual_index;
				}
				updateResiduals.push_back(updateResidual);
			}
		}

		if (i < stateVector_.size()-1) {
			for (int j=0; j<stateVector_[i]->numProcessSensors_; ++j) {
				if (stateVector_[i]->processChains_[j]->initialized_) {
					Eigen::VectorXd processResidual(stateVector_[i]->processChains_[j]->residualDimension()+2);
					processResidual(0) = stateVector_[i]->t_;
					processResidual(1) = (double)j;
					for (int k=0; k<stateVector_[i]->processChains_[j]->residualDimension(); ++k) {
						processResidual(k+2) = residuals_[residual_index];
						++residual_index;
					}
					processResiduals.push_back(processResidual);
				}
			}
		}
	}

	if (residual_index != residuals_.size())
		std::cout << "ERROR in getResiduals. residual_index=" << residual_index <<
			" but residual size is " << residuals_.size() << std::endl;
}

} // namespace confusion
