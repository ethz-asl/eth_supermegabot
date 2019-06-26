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

#ifndef CONFUSION_TRAJECTORYALIGNMENT_H_
#define CONFUSION_TRAJECTORYALIGNMENT_H_

//#include <ros/ros.h>
#include <ceres/ceres.h>
#include "confusion/utilities/Pose.h"
#include "confusion/utilities/ceres_utils.h"
#include "confusion/LocalParameterization.h"
#include "confusion/models/TrajectoryAlignmentCost.h"

namespace confusion {

class AlignTrajCallback : public ceres::IterationCallback {
 public:
  AlignTrajCallback(Eigen::Matrix<double, 6, 1> &sum_error_, Pose<double> &T_wref_wmeas_,
                    Pose<double> &T_imeas_iref_, double &delta_t_, double &scale_, Eigen::MatrixXd &errors_)
      : sum_error(sum_error_), T_wref_wmeas(T_wref_wmeas_),
        T_imeas_iref(T_imeas_iref_), delta_t(delta_t_), scale(scale_), errors(errors_) {}

  virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) {
    std::cout << "sum_error: " << sum_error.transpose() << "\n" << std::endl;
    T_wref_wmeas.print("T_wref_wmeas");
    T_imeas_iref.print("T_imeas_iref");
    std::cout << "delta_t: " << delta_t << "; scale: " << scale << std::endl;

//		if (count == 0)
//			publishIgpsErrors(states, errors);

    ++count;

    //Reset the error sums
    sum_error.setZero();

    //Allow the user to terminate the optimization if desired
//    if (!ros::ok())
//      return ceres::SOLVER_ABORT;
//    else
      return ceres::SOLVER_CONTINUE;
  }

  Eigen::Matrix<double, 6, 1> &sum_error;
  Pose<double> &T_wref_wmeas;
  Pose<double> &T_imeas_iref;
  double &delta_t;
  double &scale;
  Eigen::MatrixXd &errors;
  int count = 0;
};

//todo Make this a class with initialize, optimize, post-process, etc functions
//todo Remove the error output arguments in the cost and optionally get the residuals after solving via problem::Evaluate
/**
 * Find the spacial and temporal offsets to align two estimated trajectories. Quantities optimized for are:
 * T_wref_wmeas : Pose of the reference world frame with respect to the measurement world frame
 * T_imeas_iref : Pose of the reference target frame with respect to the measurement target frame
 * dt : Time offset between the two trajectories (t_ref = t_meas + dt)
 * scale : Scale difference between the two trajectories
 * The user can specify which parameters should be solved for. When all parameters are being solved for it is assumed
 * that a good initial value for T_imeas_iref is passed. T_wref_wmeas and dt are initialized using the start of the
 * passed trajectories.
 * @param referenceTrajectory
 * @param measuredTrajectory
 * @param optWorldFrames Flags indicating which parameters should be optimized
 * @param optTargetFrames
 * @param optTimeDelay
 * @param optScale
 * @param refDropTime Ignore measurement poses that do not have a corresponding reference pose within this neighboring time window
 * @param T_wref_wmeas Resulting world frame offset
 * @param T_imeas_iref Resulting target frame offset -- A good initial value should be passed (will be Identity when the same target frame is considered in both trajectories)
 * @param delta_t Resulting time offset -- Initialized using the first timestamp of each trajectory
 * @param scale Resulting scale factor
 * @param trans_stddev To set the relative weighting of trans/rot errors
 * @param rot_stddev
 * @param errors_ The final error values can be optionally output to this matrix for post-processing and analysis
 * @param lossCoeff Cauchy loss coefficient used for outlier rejection (optional)
 * @return referenceTrajectory with the resulting offsets applied
 */
std::vector<TrajectoryPoint> alignTrajectory(std::vector<TrajectoryPoint> &referenceTrajectory,
                                             std::vector<TrajectoryPoint> &measuredTrajectory,
                                             bool optWorldFrames,         //Specify which parameters should be solved for
                                             bool optTargetFrames,
                                             bool optTimeDelay,
                                             bool optScale,
                                             double refDropTime,          //
                                             Pose<double> &T_wref_wmeas,  //
                                             Pose<double> &T_imeas_iref,  //
                                             double &delta_t,             //
                                             double &scale,
                                             const double trans_stddev,
                                             const double rot_stddev,
                                             Eigen::MatrixXd *errors_ = NULL,
                                             double *lossCoeff = NULL) {
  //Set the measurement weightings
  double w_igps_trans = 1.0 / trans_stddev;
  double w_igps_rot = 1.0 / rot_stddev;

  //Initialize the time offset using the first timestamps from each trajectory
  delta_t = measuredTrajectory.front().t() - referenceTrajectory.front().t();

  //Initialize the world relative transformation using the first measurement
  T_wref_wmeas = referenceTrajectory.front().pose() * T_imeas_iref.inverse() * measuredTrajectory.front().pose().inverse();

  //Bins for errors
  Eigen::Matrix<double, 6, 1> sum_error_vec;
  sum_error_vec.setZero();
  Eigen::MatrixXd errors(6, measuredTrajectory.size());

  //Create the optimization problem and options
  ceres::Problem problem;
  //Set the solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.parameter_tolerance = 1e-12;
//	options.initial_trust_region_radius = 1e-2;
//	options.minimizer_type = ceres::LINE_SEARCH;
//	options.line_search_direction_type = ceres::BFGS;
//	options.linear_solver_type = ceres::DENSE_QR; //ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 200;
  options.num_threads = 1;

  AlignTrajCallback callback(sum_error_vec, T_wref_wmeas, T_imeas_iref, delta_t, scale, errors);
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true;

  //Create the alignment costs
  int numCosts = 0;
  for (int i = 0; i < measuredTrajectory.size(); i++) {
//    if ((startIndex >= 0 && i < startIndex) || (endIndex >= 0 && i >= endIndex)) {
////			std::cout << "Ignoring measurement state " << i << "; startIndex: " << startIndex << std::endl;
//      continue;
//    }

    ceres::CostFunction *cf = new
        ceres::AutoDiffCostFunction<TrajectoryAlignmentCost, 6, 3, 4, 3, 4, 3, 4, 1, 1>(new
                  TrajectoryAlignmentCost(referenceTrajectory,
                                          measuredTrajectory[i].t(),
                                          w_igps_trans,
                                          w_igps_rot,
                                          refDropTime,
                                          sum_error_vec.data(),
                                          errors.data() + 6 * i));

    if (lossCoeff) {
      problem.AddResidualBlock(cf, new ceres::CauchyLoss(*lossCoeff),
                               measuredTrajectory[i].pose().trans.data(),
                               measuredTrajectory[i].pose().rot.coeffs().data(),
                               T_wref_wmeas.trans.data(),
                               T_wref_wmeas.rot.coeffs().data(),
                               T_imeas_iref.trans.data(),
                               T_imeas_iref.rot.coeffs().data(),
                               &delta_t,
                               &scale);
    } else {
      problem.AddResidualBlock(cf, NULL,
                               measuredTrajectory[i].pose().trans.data(),
                               measuredTrajectory[i].pose().rot.coeffs().data(),
                               T_wref_wmeas.trans.data(),
                               T_wref_wmeas.rot.coeffs().data(),
                               T_imeas_iref.trans.data(),
                               T_imeas_iref.rot.coeffs().data(),
                               &delta_t,
                               &scale);
    }

    //Set the imu poses constant
    problem.SetParameterBlockConstant(measuredTrajectory[i].pose().trans.data());
    problem.SetParameterBlockConstant(measuredTrajectory[i].pose().rot.coeffs().data());

    ++numCosts;
  }

  std::cout << "Added " << numCosts << " costs to the optimization problem" << std::endl;

  if (!optWorldFrames) {
    problem.SetParameterBlockConstant(T_wref_wmeas.trans.data());
    problem.SetParameterBlockConstant(T_wref_wmeas.rot.coeffs().data());
  } else
    problem.SetParameterization(T_wref_wmeas.rot.coeffs().data(), new QuatParam);

  if (!optTargetFrames) {
    problem.SetParameterBlockConstant(T_imeas_iref.trans.data());
    problem.SetParameterBlockConstant(T_imeas_iref.rot.coeffs().data());
  } else
    problem.SetParameterization(T_imeas_iref.rot.coeffs().data(), new QuatParam);

  if (!optTimeDelay)
    problem.SetParameterBlockConstant(&delta_t);

  if (!optScale)
    problem.SetParameterBlockConstant(&scale);

  std::cout << "Error terms constructed" << std::endl;

  //Solve
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << "\n";

//	publishIgpsErrors(states, errors, bag_out);

  //Make sure that the problem is properly constrained
  Eigen::MatrixXd wOffsetCov;
  getCovariance(problem, T_wref_wmeas.trans.data(), 3, T_wref_wmeas.trans.data(), 3, "trans_wg_w", &wOffsetCov);
  std::cout << "wOffsetCov:\n" << wOffsetCov << std::endl;
//	printCovariance(problem, T_wref_wmeas.rot.coeffs().data(), 3, T_wref_wmeas.rot.coeffs().data(), 3, "rot_wg_w");
//	printCovariance(problem, T_imeas_iref.trans.data(), 3, T_imeas_iref.trans.data(), 3, "trans_i_g");
//	printCovariance(problem, T_imeas_iref.rot.coeffs().data(), 3, T_imeas_iref.rot.coeffs().data(), 3, "rot_i_g");
//	printCovariance(problem, &delta_t, 1, &delta_t, 1, "delta_t");

  //Build the output trajectory
  std::vector<TrajectoryPoint> measuredTrajectoryOut;
  for (int i = 0; i < measuredTrajectory.size(); ++i) {
    Pose<double> T_wmeas_imeas(scale * measuredTrajectory[i].pose().trans, measuredTrajectory[i].pose().rot);
    TrajectoryPoint s(measuredTrajectory[i].t() - delta_t, T_wref_wmeas * T_wmeas_imeas * T_imeas_iref);
    measuredTrajectoryOut.push_back(s);
  }

  if (errors_) {
    (*errors_) = errors;
  }

  std::cout.precision(8);
  T_wref_wmeas.print("T_wref_wmeas");
  T_imeas_iref.print("T_imeas_iref");
  std::cout.precision(15);
  std::cout << "delta_t: " << delta_t << "; scale: " << scale << std::endl;
  std::cout.precision(6);
  std::cout << std::endl;

  return measuredTrajectoryOut;
}

} // namespace confusion

#endif /* CONFUSION_TRAJECTORYALIGNMENT_H_ */
