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

#ifndef INCLUDE_CONFUSION_CONFUSOR_H_
#define INCLUDE_CONFUSION_CONFUSOR_H_

//#define OPT_DEBUG
//#define DEBUG_PRIOR
//#define PRINT_PRIOR_COST

#include <ceres/ceres.h>
#include <ros/ros.h>
#include "confusion/StateVector.h"
#include "confusion/MeasurementManager.h"
#include "confusion/ProcessChain.h"
#include "confusion/UpdateMeasurement.h"
#include "confusion/Parameter.h"
#include "confusion/State.h"
#include "confusion/StaticParameterVector.h"
#include "confusion/PriorConstraint.h"
#include "confusion/utilities/utilities.h"

namespace confusion {

/**
 * @brief This class builds and solves a Moving Horizon Estimator problem for use
 * 		in online sensor fusion.
 */
class ConFusor {
	friend class Diagram;

public:
	/**
	 * Construct a new ConFusor problem.
	 *
	 * \param firstStatePtr A pointer to an instance of the derived state is used to seed the
	 * constuction of new states in the ConFusor.
	 */
	ConFusor(std::shared_ptr<State> firstStatePtr);
	~ConFusor();

	/**
	 * Add a new process measurement to the ConFusor. It will be assigned to the
	 * appropriate state the next time assignMeasurements() is called.
	 * This can be called from a different thread than the one that calls assignMeasurements.
	 *
	 * @param processMeasurement Shared pointer to the measurement
	 */
	void addProcessMeasurement(std::shared_ptr<ProcessMeasurement> processMeasurement);
	/**
	 * Add a new update measurement to the ConFusor. It will be assigned to the
	 * appropriate state the next time assignMeasurements() is called.
	 * This can be called from a different thread than the one that calls assignMeasurements.
	 *
	 * @param updateMeasurement Shared pointer to the measurement
	 */
	void addUpdateMeasurement(std::shared_ptr<UpdateMeasurement> updateMeasurement);
	/**
	 * Add a new set of process measurements to the ConFusor. It will be assigned to the
	 * appropriate state the next time assignMeasurements() is called. The
	 * measurements can be of different types.
	 * This can be called from a different thread than the one that calls assignMeasurements.
	 *
	 * @param processMeasurements Vector of shared pointers to the measurements
	 */
	void addProcessMeasurements(std::vector<std::shared_ptr<ProcessMeasurement>> processMeasurements);
	/**
	 * Add a new set of process measurement to the ConFusor. It will be assigned to the
	 * appropriate state the next time assignMeasurements() is called. The
	 * measurements can be of different types.
	 * This can be called from a different thread than the one that calls assignMeasurements.
	 *
	 * @param updateMeasurements Vector of shared pointers to the measurements
	 */
	void addUpdateMeasurements(std::vector<std::shared_ptr<UpdateMeasurement>> updateMeasurements);

	/**
	 * @brief Set the solver options for the MHE optimization problems
	 *
	 * @param numThreads Number of threads used for cost function computations. Regardless of this value, only one thread is used for the final optimization computations, though.
	 * @param maxIterations Maximum number of iterations allowed for the solver.
	 * @param minimizerProgressToStdout Specify if Ceres should print the result of each solver iteration
	 */
	//todo Pass the full ceres options struct to give more flexibility?
	void setSolverOptions(int numThreads, int maxIterations, bool minimizerProgressToStdout = false);

	/**
	 * @brief Create new states and assign measurements to the states
	 *
	 * This should be called by the estimation thread to first set up the problem structure
	 * such that additional logic can be used to manage the optimized states and
	 * parameters.
	 *
	 * @return Returns true if a new state was created
	 */
	bool assignMeasurements();

	/**
	 * @brief Get the covariance between two estimated parameters after solving
	 * @param param1
	 * @param param1_size
	 * @param param2
	 * @param param2_size
	 * @param ss
	 * @param cov_out
	 * @return True if the covariance was successfully computed
	 */
	bool checkCovariance(const double* param1, const size_t param1_size,
		const double* param2, size_t param2_size, std::string ss,
		Eigen::MatrixXd* cov_out = nullptr);

	int numStates() { return stateVector_.size(); }

	/**
	 * @brief Add a static parameter to the current MHE problem. The static parameter
	 * will only be activated once it appears in one of the residual functions. If
	 * there is some prior confidence in the parameter value, indicate this by first
	 * calling Parameter::setInitialConstraintWeighting on the passed parameter.
	 * @param param The Parameter to be added to the MHE problem
	 */
	void addStaticParameter(Parameter param);

	/**
	 * @brief Attach a random walk process to the Parameter at the specified address. This random walk process comes
	 * between the PriorConstraint and the optimized parameter to model changes in it over time.
	 * @param data Pointer to the first element of the underlying parameter data vector
	 * @param processNoise The desired process white noise. Has units [unit of parameter / sqrt(sec)].
	 */
	void attachStaticParameterRandomWalkProcess(double* data, const double &processNoise);

	/**
	 * @brief Remove the random walk process from the Parameter with the specified data address.
	 * @param data Pointer to the first element of the desired parameter data
	 */
	void detachStaticParameterRandomWalkProcess(double* data);

	/**
	 * @brief Solve the MHE problem. ConFusor::assignMeasurements should be called
	 * between calls to ConFusor::optimize to incorporate new measurements and add
	 * new states.
	 */
	void optimize();

	/**
	 * @brief Get the values of all of the residulas in the current MHE problem
	 * given the current value of the estimated parameters. ConFusor::optimize
	 * must have been called previously to build the MHE problem.
	 *
	 * @param priorResidual Computed prior residual vector returned here
	 * @param processResiduals Computed process residuals returned here
	 * @param updateResiduals Computed update residuals returned here
	 */
	void getResiduals(Eigen::VectorXd& priorResidual,
			std::vector<Eigen::VectorXd>& processResiduals,
			std::vector<Eigen::VectorXd>& updateResiduals);

	/**
	 * @brief Get the values of all of the residulas in the current MHE problem
	 * given the current value of the estimated parameters. ConFusor::optimize
	 * must have been called previously to build the MHE problem.
	 *
	 * @param priorResidual Computed prior residual vector returned here
	 * @param processResiduals Computed process residuals returned here
	 * @param updateResiduals Computed update residuals returned here
	 * @param randomWalkProcessResiduals Computed residuals for the random walk processes attached to static parameters
	 */
	void getResiduals(Eigen::VectorXd& priorResidual,
					std::vector<Eigen::VectorXd>& processResiduals,
					std::vector<Eigen::VectorXd>& updateResiduals,
					std::vector<Eigen::VectorXd>& randomWalkProcessResiduals);

	/**
	 * @brief Marginalize the desired number of state from the front of the active
	 * batch of states. At least one state needs to remain active after marginalization.
	 *
	 * @param numStatesToMarginalize The number of states to marginalize
	 * @param statesOut Optionally output the marginalized states to be stored for use later, e.g. in a batch calibration (BatchFusor) problem
	 */
	void marginalizeFrontStates(int numStatesToMarginalize, StateVector* statesOut = nullptr);

	/**
	 * @brief Set the desired static parameters constant
	 *
	 * The desired parameters will be marginalized out of the PriorConstraint and
	 * their values will be fixed over subsequent MHE problems.
	 *
	 * @param parameters The static parameters to be set constant
	 */
	void setStaticParamsConstant(std::vector<double*> parameters);

	/**
	 * @brief Remove the desired parameters from the MHE problem
	 *
	 * This first calls
	 * ConFusor::setStaticParamsConstant then removes the parameters altogether.
	 * Note that it is up to the user to ensure that any active cost functions do not try
	 * to use these parameters after they have been removed.
	 *
	 * @param parameters The static parameters to be removed
	 */
	void removeStaticParameters(std::vector<double*> parameters);

	/**
	 * @brief Remove a state from the batch by throwing out the update
	 * measurements and merging the process measurements with the previous state.
	 *
	 * This can be used to increase the temporal spread of a batch of states
	 * of limited size. Optimizing all new states at least once can be beneficial
	 * to effectively cancel any drift, and then this function can be used to
	 * maintain a small batch of states.
	 *
	 * \param index The index of the state to be removed
	 */
	void removeIntermediateState(int index);

	/**
	 * @brief Perform some sanity checks on the structure of the current MHE problem
	 * @return Return true if everything looks ok
	 */
	bool checkProblemSetup();

	/**
	 * The the iteration callback for the underlying ceres optimization problem.
	 * A common choice is an instance of confusion::SolverOverrunCallback, which
	 * allows the user to control when an optimization should be aborted due to
	 * timing overruns. See the ceres documentation for details on IterationCallbacks.
	 *
	 * @param callback Pointer to the desired iteration callback
	 */
	void setIterationCallback(std::shared_ptr<ceres::IterationCallback> callback) {
		callback_ = callback;
		solverOptions_.callbacks.push_back(callback_.get());
	}

	/**
	 * Print detailed information about the last optimization problem to the
	 */
	void verbosePrint();
	/**
	 * Print minimal information about the last optimization problem to the
	 */
	void briefPrint();

	/**
	 * @brief Remove all states from the MHE problem, essentially stopping tracking
	 * while maintaining the current confidence in the set of static parameters.
	 *
	 * Tracking can then be restarted by simply passing new measurements and calling
	 * ConFusor::assignMeasurements and ConFusor::optimize.
	 *
	 * @param statesOut Optionally output all of the previously active states for
	 * logging or use in a future batch calibration (BatchFusor) problem.
	 */
	void stopTracking(StateVector* statesOut = nullptr);

	/**
	 * Get the termination type from the last MHE optimization. This can be used
	 * to check if the last optimization overran it its time limit.
	 *
	 * @return The last optimization's termination type
	 */
	ceres::TerminationType getLastTerminationType() {
		return summary_.termination_type;
	}

	/**
	 * [FOR DEVEL ONLY] Set all parameters of the first state and the static parameters constant. This is intended
	 * to be used in the a high-rate tracking thread, alongside a lower rate mapping thread, similar to in ORB_SLAM.
	 */
	void fixFirstStateAndStaticParameters();

	size_t priorConstraintSize() const { return priorConstraint_.priorCostWeighting_.rows(); }


	StateVector stateVector_;
	StaticParameterVector staticParameters_;

	Eigen::MatrixXd stateInitialWeighting_; //Store this so that tracking can be stopped and restarted

    ceres::Solver::Options solverOptions_;

private:
//	void setMarginalizedParams();

	void buildProblem();

    //Return the local size of the parameters to be marginalized
    int buildPriorProblem(size_t numStatesToMarginalize, std::vector<double*> &priorParameterBlocks);

	ceres::Problem* problem_ = nullptr;
	ceres::Problem::Options problemOptions_;

	std::shared_ptr<ceres::IterationCallback> callback_;
	ceres::Solver::Summary summary_;
	std::vector<FixedParameter> marginalizedParams_;

	MeasurementManager measurementManager_;

    PriorConstraint priorConstraint_;

	Eigen::VectorXd lastPriorError_;

	int numThreads_ = 4;
	int maxIterations_ = 20;
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_CONFUSOR_H_ */
