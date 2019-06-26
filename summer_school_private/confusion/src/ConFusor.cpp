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

#include "confusion/ConFusor.h"

namespace confusion {

ConFusor::ConFusor(std::shared_ptr<State> firstStatePtr):
		measurementManager_(firstStatePtr->numProcessSensors_, firstStatePtr->numUpdateSensors_),
		stateVector_(firstStatePtr)	{
	//Configure the ceres problem now to take ownership of these things because they are reused as long as a state remains in the batch
	problemOptions_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problemOptions_.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problemOptions_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

	//todo These need to be configurable by the user
//	solverOptions_.minimizer_progress_to_stdout = true;
//	options.initial_trust_region_radius = 1e-2;
//	options.minimizer_type = ceres::LINE_SEARCH;
//	options.line_search_direction_type = ceres::BFGS;
//	options.linear_solver_type = ceres::DENSE_QR; //ceres::SPARSE_NORMAL_CHOLESKY;
//	solverOptions_.max_num_iterations = maxIterations_;
//	solverOptions_.num_threads = numThreads_;
}


void ConFusor::setSolverOptions(int numThreads, int maxIterations, bool minimizerProgressToStdout) {
	maxIterations_ = maxIterations;
	numThreads_ = numThreads;
	solverOptions_.max_num_iterations = maxIterations_;
	solverOptions_.num_threads = numThreads_;
    solverOptions_.minimizer_progress_to_stdout = minimizerProgressToStdout;
}


ConFusor::~ConFusor() {
	if(problem_)
		delete problem_;
}


void ConFusor::addProcessMeasurement(std::shared_ptr<ProcessMeasurement> processMeasurement) {
	measurementManager_.addProcessMeasurement(processMeasurement);
}


void ConFusor::addUpdateMeasurement(std::shared_ptr<UpdateMeasurement> updateMeasurement) {
	measurementManager_.addUpdateMeasurement(updateMeasurement);
}


void ConFusor::addProcessMeasurements(std::vector<std::shared_ptr<ProcessMeasurement>> processMeasurements) {
	for (const auto& processMeas: processMeasurements)
		measurementManager_.addProcessMeasurement(processMeas);
}


void ConFusor::addUpdateMeasurements(std::vector<std::shared_ptr<UpdateMeasurement>> updateMeasurements) {
	for (const auto& updateMeas: updateMeasurements)
		measurementManager_.addUpdateMeasurement(updateMeas);
}


bool ConFusor::assignMeasurements() {
	return measurementManager_.assignMeasurements(stateVector_, staticParameters_);
}


void ConFusor::addStaticParameter(Parameter param) {
	//Add the new parameter
	staticParameters_.addParameter(param);
}

void ConFusor::attachStaticParameterRandomWalkProcess(double* data, const double &processNoise) {
	staticParameters_.attachStaticParameterRandomWalkProcess(data, processNoise);
}

void ConFusor::detachStaticParameterRandomWalkProcess(double* data) {
  staticParameters_.detachStaticParameterRandomWalkProcess(data);

	// This is required to make sure the parameter linked to the prior constraint is updated
	//todd Can this be done in a nicer way?
  priorConstraint_.fixMarginalizedParams();
}


bool ConFusor::checkProblemSetup() {
	bool res = true;
	if (!stateVector_.check())
		res = false;
	if (!staticParameters_.check())
		res = false;

	if (!priorConstraint_.check())
		res = false;

	return res;
}


bool ConFusor::checkCovariance(const double* param1, const size_t param1_size,
		const double* param2, size_t param2_size, std::string ss, Eigen::MatrixXd* cov_out) {
	if (problem_) {
      if (getCovariance(*problem_, param1, param1_size, param2, param2_size, ss, cov_out))
        return true;
    }
	else
		std::cout << "ERROR: User must call optimize() before checkCovariance!" << std::endl;

	return false;
}

void ConFusor::buildProblem() {
  //Remove the old problem and set up the new one
  //todo Use the same problem all of the time? How much can be retained?
  if(problem_)
    delete problem_;
  problem_ = new ceres::Problem(problemOptions_);

	//Start with all static parameters deactivated and activate them as they appear linked to measurements
	stateVector_.deactivateParameters();
	staticParameters_.deactivateParameters();

	for(int i=0; i<stateVector_.size(); ++i) {
		int numUpdateCosts = 0;
		int numProcessCosts = 0;

		//Add update residual blocks
		for (int j=0; j<stateVector_[i]->numUpdateSensors_; ++j) {
			for (int k=0; k<stateVector_[i]->updateMeasurements_[j].size(); ++k) {
#ifdef OPT_DEBUG
				std::cout << "State at " << stateVector_[i]->t_ << ". Adding update cost type " << j << " at index " << k << std::endl;
#endif
				//todo Check nullptr even though it should never be null?
				if (stateVector_[i]->updateMeasurements_[j][k]->addCostToProblem(problem_,
						stateVector_[i]->parameters_, staticParameters_)) {
					++numUpdateCosts;
				}
			}
		}

		//Add process update residual blocks
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

  //Initialize the prior constraint if this is the first problem
  if (!priorConstraint_.initialized()) {
    //Set up the prior constraint for the first problem
    for (int i=0; i<stateVector_.front()->parameters_.size(); ++i) {
      if (stateVector_.front()->parameters_[i].isActive() &&
          stateVector_.front()->parameters_[i].immediatelyAddToPrior() &&
          !stateVector_.front()->parameters_[i].priorConstraintActive()) {
//std::cout << "initializing state parameter prior constraint. i=" << i << std::endl;
        priorConstraint_.addStateParameterAndActivate(stateVector_.front()->parameters_[i]);
      }
//			else {
//std::cout << "no prior constraint active for state parameter" << std::endl;
//			}
    }

    priorConstraint_.fixMarginalizedParams();
    priorConstraint_.initialize();
  }

  //Check if there are static parameters that need to be immediately added to the prior constraint
  for (auto& iter: staticParameters_) {
    if (iter.second.isActive() &&
        iter.second.immediatelyAddToPrior() &&
        !iter.second.priorConstraintActive() &&
        !iter.second.isConstant()) {
      //Add the parameter to the prior constraint the first time it goes active
      //todo Make this a separate function in the prior constraint?
      priorConstraint_.addStaticParameter(iter.second);
#ifdef OPT_DEBUG
      std::cout << "Added static parameter with size " << iter.second.size() <<
					" to the problem and immediately activated it." << std::endl;
#endif
    }
  }

  //Create and add the prior cost
  //This also adds any random walk processes linked to the static parameters to the problem
  priorConstraint_.addPriorCostToProblem(problem_);

  //Add the active state parameters to the problem
  for(int i=0; i<stateVector_.size(); ++i) {
    int numParameters = 0;
    for (int j=0; j<stateVector_[i]->parameters_.size(); ++j) {
      if (stateVector_[i]->parameters_[j].isActive()) {
//std::cout << "Adding state parameter. i=" << i << "; j=" << j << std::endl;
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
//	for (int i=0; i<staticParameters_.numParameters(); ++i) {
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

void ConFusor::optimize() {
	//Check the problem setup
	//todo Fail gracefully if this fails?
	if (!checkProblemSetup()) {
      std::cout << "ERROR: ConFusor::checkProblemSetup failed!" << std::endl;
	}

	//Add all measurement cost functions
	buildProblem();

#ifdef OPT_DEBUG
	std::cout << "Optimizing " << stateVector_.size() << " states, over a " << stateVector_.back()->t_ - stateVector_.front()->t_ <<
			" time horizon, and " << staticParameters_.numParameters() << " static parameters" << std::endl;
#endif

	//Solve
	Solve(solverOptions_, problem_, &summary_);

	//Print if there was an error solving
	if (summary_.termination_type == ceres::FAILURE || summary_.termination_type == ceres::USER_FAILURE) {
		std::cout << "ConFusor::optimize ERROR: Solving the MHE problem failed! Terminated with message: " << summary_.message << std::endl;
	}
}

void ConFusor::verbosePrint() {
	std::cout << summary_.FullReport() << "\n";
}

void ConFusor::briefPrint() {
//	std::cout << summary_.BriefReport() << "\n";
	std::cout << "ConFusor took " << summary_.num_successful_steps << " of " <<
			summary_.num_unsuccessful_steps+summary_.num_successful_steps <<
			" successful steps. Final cost: " << summary_.final_cost << std::endl;
}

void ConFusor::getResiduals(Eigen::VectorXd& priorResidual,
                            std::vector<Eigen::VectorXd>& processResiduals,
                            std::vector<Eigen::VectorXd>& updateResiduals) {
  std::vector<Eigen::VectorXd> randomWalkProcessResiduals;
  getResiduals(priorResidual, processResiduals, updateResiduals, randomWalkProcessResiduals);
}

void ConFusor::getResiduals(Eigen::VectorXd& priorResidual,
                  std::vector<Eigen::VectorXd>& processResiduals,
                  std::vector<Eigen::VectorXd>& updateResiduals,
                  std::vector<Eigen::VectorXd>& randomWalkProcessResiduals) {
	if (!problem_) {
		std::cerr << "ERROR: There no previous problem available for residual computation! You need to optimize before calling getResiduals." << std::endl;
		return;
	}

	ceres::Problem::EvaluateOptions evalOptions;
	evalOptions.num_threads = numThreads_;

	double cost;
	std::vector<double> residuals;
//	std::vector<double> gradient;
//	ceres::CRSMatrix priorJacobian_crs;
	problem_->Evaluate(evalOptions, &cost, &residuals, nullptr, nullptr); //, &gradient, &priorJacobian_crs);

//	std::cout << "getResiduals computed cost=" << cost << " from " << residuals.size() << " residuals" << std::endl;

	Eigen::Map<Eigen::VectorXd> residuals_(residuals.data(), residuals.size());
//std::cout << "residuals: " << residuals_.transpose() << std::endl;

	//Be careful. Here the order of the residual blocks is assumed to be consistent with how they are added to the problem in State::addCostsToProblem
	int residual_index = 0;
	for (int i=0; i<stateVector_.size(); ++i) {
		for (int j=0; j<stateVector_[i]->numUpdateSensors_; ++j) {
			for (int k=0; k<stateVector_[i]->updateMeasurements_[j].size(); ++k) {
			  if (stateVector_[i]->updateMeasurements_[j][k]->initialized() && stateVector_[i]->updateMeasurements_[j][k]->isEnabled()) {
                Eigen::VectorXd updateResidual(stateVector_[i]->updateMeasurements_[j][k]->residualDimension() + 2);
                updateResidual(0) = stateVector_[i]->t_;
                updateResidual(1) = (double) j;
                for (int l = 0; l < stateVector_[i]->updateMeasurements_[j][k]->residualDimension(); ++l) {
                  updateResidual(l + 2) = residuals_[residual_index];
                  ++residual_index;
                }
                updateResiduals.push_back(updateResidual);
              }
			}
		}

		if (i < stateVector_.size()-1) {
			for (int j=0; j<stateVector_[i]->numProcessSensors_; ++j) {
				if (stateVector_[i]->processChains_[j]->initialized() && stateVector_[i]->processChains_[j]->isEnabled()) {
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

	// Add the prior residual
	priorResidual.resize(priorConstraint_.priorErrorOffset_.size()+1);
	priorResidual(0) = stateVector_.front()->t_;
	for (int i=0; i<priorConstraint_.priorErrorOffset_.size(); ++i) {
		priorResidual(i+1) = residuals_[residual_index];
		++residual_index;
	}

	// Add the static parameter random walk processes
    for (const auto& p: priorConstraint_.staticPriorParameters_) {
      if (p->isRandomWalkProcessActive()) {
        Eigen::VectorXd rwpResidual(p->localSize());
        for (int i=0; i<p->localSize(); ++i) {
          rwpResidual(i) = residuals_[residual_index];
          ++residual_index;
        }
        randomWalkProcessResiduals.push_back(rwpResidual);
      }
    }

	if (residual_index != residuals_.size())
		std::cout << "ERROR in ConFusor::getResiduals. residual_index=" << residual_index <<
			" but residual size is " << residuals_.size() << std::endl;
}

int ConFusor::buildPriorProblem(size_t numStatesToMarginalize, std::vector<double*> &priorParameterBlocks) {
  //Remove the old problem and set up the new one
  if(problem_)
    delete problem_;
  problem_ = new ceres::Problem(problemOptions_);

  if (numStatesToMarginalize >= stateVector_.size()) {
    std::cout << "ERROR: ConFusor::buildPriorProblem requested for too many states!" << std::endl;
    numStatesToMarginalize = stateVector_.size() - 1;
  }

  //For maginalization, we build the prior problem from scratch, but keep the prior constraint from the previous marginalization.

#ifdef OPT_DEBUG
  std::cout << "---Building the prior problem to marginalize " << numStatesToMarginalize << " states---" << std::endl;
#endif

  //Start with all static parameters deactivated and activate them as they appear linked to measurements
  stateVector_.deactivateParameters();
  staticParameters_.deactivateParameters();

  //Add the prior constraint from the previous problem first, since we will rebuild the prior constraint here as well
  if (!priorConstraint_.initialized()) {
    std::cout << "ERROR: Marginalization cannot be performed before the PriorConstraint has been initialized!" << std::endl;
    abort();
  }
  //Get the difference in time between the current and next front state. This is used to update the static parameter RWPs.
  double dtFrontState = stateVector_[numStatesToMarginalize]->t() - stateVector_.front()->t();

  //Add the last prior
  priorConstraint_.addPriorCostToProblem(problem_, &dtFrontState);

  //Add costs connected to the states to be marginalized
  for(int i=0; i<numStatesToMarginalize; ++i) {
    int numUpdateCosts = 0;
    int numProcessCosts = 0;

    //Add update residual blocks
    for (int j=0; j<stateVector_[i]->numUpdateSensors_; ++j) {
      for (int k=0; k<stateVector_[i]->updateMeasurements_[j].size(); ++k) {
#ifdef OPT_DEBUG
        std::cout << "State at " << stateVector_[i]->t_ << ". Adding update cost type " << j << " at index " << k << std::endl;
#endif
        //todo Check nullptr even though it should never be null?
        ceres::ResidualBlockId residualBlockId;
        if (stateVector_[i]->updateMeasurements_[j][k]->addCostToProblem(problem_,
                                                                         stateVector_[i]->parameters_, staticParameters_)) {
          ++numUpdateCosts;
        }
      }
    }

    //Add process update residual blocks
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

#ifdef OPT_DEBUG
    std::cout << "Added " << numUpdateCosts << " update costs and " <<
				numProcessCosts << " process costs for the state at " << stateVector_[i]->t_ << std::endl;
#endif
  }

  // We need to explicitly specify the parameter ordering to make sure it is correct for marginalizing the first states' parameters
  // Ordering for e.g. marginalizing two states is: [x0, x1, s_rwp, x2, s_marg, s_nonmarg]
  priorParameterBlocks.clear();

  // Add the state parameters for the states to be marginalized
  // Keep track of the parameter sizes
  // Add the active parameters of the new leading state to the next priorConstraint
  int margParamLocalSize = 0;
  int stayParamLocalSize = 0;
  priorConstraint_.clearStateParameters();
  for(int i=0; i<numStatesToMarginalize; ++i) {
    int numParameters = 0;
    for (int j=0; j<stateVector_[i]->parameters_.size(); ++j) {
      if (stateVector_[i]->parameters_[j].isActive() &&
			  !stateVector_[i]->parameters_[j].isConstant()) {
//std::cout << "Adding state parameter. i=" << i << "; j=" << j << std::endl;
        stateVector_[i]->parameters_[j].addToProblem(problem_);

        margParamLocalSize += stateVector_[i]->parameters_[j].localSize();

        priorParameterBlocks.push_back(stateVector_[i]->parameters_[j].data_);
        ++numParameters;
      }
    }
#ifdef OPT_DEBUG
    std::cout << "Added " << numParameters << " of " << stateVector_[i]->parameters_.size() <<
				" parameters for the state at " << stateVector_[i]->t_ << std::endl;
#endif
  }

  // Add prior-side rwp-contrained static parameters (cost functions were already added along with the prior constraint)
  int numRwpParameters = 0;
  for (const auto& p: priorConstraint_.staticPriorParameters_) {
    if (p->isRandomWalkProcessActive()) {
      priorParameterBlocks.push_back(p->rwpPriorSideParameterAddress());
      margParamLocalSize += p->localSize();
      ++numRwpParameters;
    }
  }
#ifdef OPT_DEBUG
  if (numRwpParameters > 0)
    std::cout << "Added " << numRwpParameters << " rwp prior-side parameters to the prior problem" << std::endl;
#endif

  // Add the parameters of the next leading state and also add them to the new priorConstraint
  int numParameters = 0;
  for (int j=0; j<stateVector_[numStatesToMarginalize]->parameters_.size(); ++j) {
    if (stateVector_[numStatesToMarginalize]->parameters_[j].isActive() &&
        !stateVector_[numStatesToMarginalize]->parameters_[j].isConstant()) {
//std::cout << "Adding state parameter. i=" << i << "; j=" << j << std::endl;
      stateVector_[numStatesToMarginalize]->parameters_[j].addToProblem(problem_);

      stayParamLocalSize += stateVector_[numStatesToMarginalize]->parameters_[j].localSize();
      priorConstraint_.addStateParameter(stateVector_[numStatesToMarginalize]->parameters_[j]);

      priorParameterBlocks.push_back(stateVector_[numStatesToMarginalize]->parameters_[j].data_);
      ++numParameters;
    }
  }
#ifdef OPT_DEBUG
  std::cout << "Added " << numParameters << " of " << stateVector_[numStatesToMarginalize]->parameters_.size() <<
				" parameters for the new leading state at " << stateVector_[numStatesToMarginalize]->t_ << std::endl;
#endif

  //Add any new active static parameters to the prior constraint (not yet to the parameter block vector though)
  //The prior constraint maintains the static parameter ordering from the order in which the params were added to it
  for (auto& iter: staticParameters_) {
    if (iter.second.isActive()) {
      iter.second.addToProblem(problem_);
      bool addToPriorConstraint = false;
      if (!iter.second.isConstant() && !iter.second.priorConstraintActive()) {
        priorConstraint_.addStaticParameter(iter.second);
        addToPriorConstraint = true;
      }
#ifdef OPT_DEBUG
      std::cout << "Added static parameter with size " <<
                iter.second.size() << ". addToPriorConstraint=" << addToPriorConstraint << std::endl;
#endif
    }
  }

  //Add all static parameters in the prior problem without rwps attached at the end of the parameter vector
  for (const auto& p: priorConstraint_.staticPriorParameters_) {
//    if (!p->isRandomWalkProcessActive()) {
      priorParameterBlocks.push_back(p->data_);
//    }
  }

  return margParamLocalSize;
}


void ConFusor::marginalizeFrontStates(int numStatesToMarginalize, StateVector* statesOut) {
  if (numStatesToMarginalize <= 0)
    return;

	if (!problem_) {
		std::cout << "ERROR: There no previous problem available for marginalization. You must optimize before marginalizing." << std::endl;
		return;
	}
	//todo Also allow to marginalize out all states? Just need to handle the added logic for the residuals of the last state
	if (numStatesToMarginalize >= stateVector_.size()) {
		std::cout << "ERROR: It is not supported to marginalize all states. Marginalizing out all but one states." << std::endl;
		numStatesToMarginalize = stateVector_.size()-1;
	}

	//Copy out the marginalized states if desired
	if (statesOut) {
		for (int i=0; i<numStatesToMarginalize; ++i)
			statesOut->pushState(stateVector_[i]);
	}

	//Build the prior problem
	std::vector<double*> priorParameterBlocks;
	int margParamLocalSize = buildPriorProblem(numStatesToMarginalize, priorParameterBlocks);

	if (margParamLocalSize > 0) {
		ceres::Problem::EvaluateOptions evalOptions;
		evalOptions.num_threads = numThreads_;
		evalOptions.parameter_blocks = priorParameterBlocks;

		double cost;
		std::vector<double> residuals;
		ceres::CRSMatrix priorJacobian_crs;
		problem_->Evaluate(evalOptions, &cost, &residuals, nullptr, &priorJacobian_crs);

#ifdef DEBUG_PRIOR
        std::cout << "Evaluting the previous problem generated " << residuals.size() <<
                " residuals, and jacobian of size " << priorJacobian_crs.num_rows << "," <<
                priorJacobian_crs.num_cols << ". Will factor out the top-left portion of size " << margParamLocalSize << std::endl;
#endif

		Eigen::VectorXd priorResidual(residuals.size());
		for (int i = 0; i < residuals.size(); ++i)
			priorResidual(i) = residuals[i];

		Eigen::MatrixXd priorJacobian;
		buildDenseMatrix(priorJacobian_crs, priorJacobian);
//std::cout << "priorJacobian for factoring:\n" << priorJacobian.topLeftCorner(margParamLocalSize,margParamLocalSize) << std::endl;
		Eigen::MatrixXd H = priorJacobian.transpose() * priorJacobian;
		Eigen::VectorXd b = priorJacobian.transpose() * priorResidual;

		schurMarginalize(H, b, margParamLocalSize,
						 priorConstraint_.priorCostWeighting_, priorConstraint_.priorErrorOffset_,
						 &priorConstraint_.Hstar_, &priorConstraint_.bstar_);
//std::cout << "priorJ is: " << priorJacobian.rows() << "'" << priorJacobian.cols() << "; H: "  << H.rows() << "'" << H.cols() << "; Hstar_lastStateMarg_: "   << priorConstraint_.Hstar_.rows() << "'" << priorConstraint_.Hstar_.cols() << std::endl;
//std::cout << "J_prior:\n" << priorJacobian << std::endl;
//std::cout << "priorCostWeighting_:\n" << priorCostWeighting_ << std::endl;
//std::cout << "priorErrorOffset_:\n" << priorErrorOffset_.transpose() << std::endl;
	}
	else {
		//No parameters to marginalize
		std::cout << "WARNING: No state paramters to add to the prior constraint? Skipping marginalization, but something is probably wrong." << std::endl;
		priorConstraint_.clearStateParameters(); //To clear the
	}

	//Remove the states
	for (int i=0; i<numStatesToMarginalize; ++i)
		stateVector_.popState();

	priorConstraint_.fixMarginalizedParams();

    if (!priorConstraint_.check()) {
      std::cout << "ERROR: PriorConstraint::check failed after building the prior problem" << std::endl;
      priorConstraint_.print();
    }
}

void ConFusor::removeIntermediateState(int index) {
//	std::cout << "Removing state " << index << " of " << stateVector_.size() <<
//			" at " << stateVector_[index]->t_ << std::endl;

	if (index <= 0 || index >= stateVector_.size()-1)
		std::cout << "ERROR: User requested to remove an invalid state. The first and last states cannot be removed using this function." << std::endl;

	//Reset the process chains of the previous state and merge the measurements
	//with those from the state being removed
	for (int i=0; i<stateVector_[index-1]->numProcessSensors_; ++i) {
		std::deque<std::shared_ptr<ProcessMeasurement>> measurements = stateVector_[index-1]->processChains_[i]->measurements_;
		measurements.insert(measurements.end(),
				stateVector_[index]->processChains_[i]->measurements_.begin(),
				stateVector_[index]->processChains_[i]->measurements_.end());

		bool wasReady = stateVector_[index]->processChains_[i]->ready();

		//Reset the processChain from the previous state and add the new measurements
		stateVector_[index-1]->processChains_[i]->reset();
		stateVector_[index-1]->processChains_[i]->measurements_ = measurements;

		//If the processChain of the state to be removed is already in use,
		//assign the new state parameters to the last state's processChain
		if (wasReady) {
			stateVector_[index-1]->processChains_[i]->assignTimes(
					stateVector_[index-1]->t(), stateVector_[index+1]->t());
		}
	}

	//Remove the desired state
	stateVector_.removeState(index);

	checkProblemSetup();
}

void ConFusor::stopTracking(StateVector* statesOut) {
	//Optionally copy the current states over for use later
	if (statesOut) {
		//We don't copy out the most recent state because it is reset so that tracking can be restarted when appropriate
		//todo This is only because I need to keep a derived state instance in the StateVector. Better way to do this? Template StateVector on the derived type?
		for (int i=0; i<stateVector_.size()-1; ++i)
			statesOut->pushState(stateVector_[i]);
	}

	//Do some sanity checks on sizes
	priorConstraint_.check();

	//Remove the state portion of the prior constraint
	Eigen::MatrixXd Hstar_temp;
	Eigen::VectorXd bstar_temp;
	schurMarginalize(priorConstraint_.Hstar_, priorConstraint_.bstar_, priorConstraint_.statePortionLocalSize_,
			priorConstraint_.priorCostWeighting_, priorConstraint_.priorErrorOffset_,
			&Hstar_temp, &bstar_temp);
//std::cout << "Stopped tracking. W:\n" << priorConstraint_.priorCostWeighting_ << "\n priorErrorOffset: " << priorConstraint_.priorErrorOffset_.transpose() << std::endl;
	priorConstraint_.Hstar_ = Hstar_temp;
	priorConstraint_.bstar_ = bstar_temp;

//	std::cout << "Prior constraint after stopping tracking:\n priorErrorOffset: " << priorConstraint_.priorErrorOffset_.transpose() <<
//			"\npriorCostWeighting:\n" << priorConstraint_.priorCostWeighting_ << std::endl;

	//Reset the state portion of the prior constraint
	priorConstraint_.clearStateParameters();
	priorConstraint_.reset();

	//Remove all states but the most recent one and signal that the first state should be initialized
	stateVector_.reset();

	//Remove all of the pending measurements from the MeasurementManager
	measurementManager_.reset();

	std::cout << "---------Stopped tracking-------\n\n" << std::endl;

	priorConstraint_.check();
}

void ConFusor::setStaticParamsConstant(std::vector<double*> parameterAddresses) {
	std::vector<std::pair<int,int>> segmentPositionAndSize;
	int dimToMarginalize = 0;
	std::vector<Parameter*> parametersForMarg;
	for (auto& parameterAddress : parameterAddresses) {
		Parameter* parameter = staticParameters_.getParameter(parameterAddress);

		if (!parameter) {
			std::cout << "User requested to set an unknown parameter constant! Ignoring this and moving on." << std::endl;
			continue;
		}

		//We can directly set the parameter constant if it isn't yet in the prior constraint
		if (!parameter->priorConstraintActive()) {
            parameter->setConstant();
			continue;
		}

		//Get the location and size of the parameter in the prior constraint
		segmentPositionAndSize.push_back(std::make_pair(parameter->priorConstraintLocalPosition() + priorConstraint_.statePortionLocalSize_, parameter->localSize()));
		dimToMarginalize += parameter->localSize();
//std::cout << "Removing static parameter of size " << parameter->localSize() << " in position " << parameter->priorConstraintLocalPosition() + priorConstraint_.statePortionLocalSize_ << std::endl;

		//Store the parameters being marginalized so we can go back and remove them from the prior constraint when we're done
		parametersForMarg.push_back(parameter);
	}

  if (parametersForMarg.empty()) //We don't need to manipulate the prior constraint
    return;

//std::cout << "H before:\n" << priorConstraint_.Hstar_ << std::endl;
		//Reorder the prior constraint matrices to move the parameter to the front
		Eigen::MatrixXd H = priorConstraint_.Hstar_;
		Eigen::VectorXd b = priorConstraint_.bstar_;
		reorderNormalEquations(H, b, segmentPositionAndSize);

		//Remove the static params through Gaussian elimination
		//todo We could give the user the option to marginalize the other way to get the marginal confidence of the parameter being removed
		Eigen::VectorXd priorErrorOffset;
		Eigen::MatrixXd priorCostWeighting;
		schurMarginalize(H, b, dimToMarginalize,
				priorConstraint_.priorCostWeighting_, priorConstraint_.priorErrorOffset_,
				&priorConstraint_.Hstar_, &priorConstraint_.bstar_);
//std::cout << "H after:\n" << priorConstraint_.Hstar_ << std::endl;

	for (auto& parameter : parametersForMarg) {
		//Remove the parameter from the prior constraint
//std::cout << "Removing static parameter of size " << parameter->localSize() << " in position " << parameter->priorConstraintLocalPosition() << std::endl;
		priorConstraint_.removeStaticParameter(*parameter);
        parameter->setConstant();
	}

	priorConstraint_.fixMarginalizedParams();
	priorConstraint_.check();
}

//Note that the measurements will need to handle the removal of these parameters themselves. They will not be notified of the removal.
void ConFusor::removeStaticParameters(std::vector<double*> parameterAddresses) {
	//Make sure the parameters are not in the prior constraint
	setStaticParamsConstant(parameterAddresses);

	for (auto& parameter: parameterAddresses)
		staticParameters_.removeParameter(parameter);
}

void ConFusor::fixFirstStateAndStaticParameters() {
	std::cout << "Fixing the prior optimized parameters in the ConFusor" << std::endl;

	//Clear the prior constraint
	priorConstraint_.completeReset();

	//Set all static parameters constant
	for (auto &staticParam: staticParameters_) {
		staticParam.second.setConstant();
	}

	//Set the leading state parameters constant
	for (auto &stateParam: stateVector_.front()->parameters_)
		stateParam.setConstant();

	//Deactivate the update measurements on the fixed state
	for (int i=0; i<stateVector_.front()->numUpdateSensors(); ++i) {
		for (auto &updateMeas: stateVector_.front()->updateMeasurements_[i])
			updateMeas->disable();
	}
}


} // namespace confusion
