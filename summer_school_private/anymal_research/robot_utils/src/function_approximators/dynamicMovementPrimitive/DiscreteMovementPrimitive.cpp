/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \author  Peter Pastor, Peter Fankhauser

 **********************************************************************/

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/DiscreteMovementPrimitive.hpp"
#include "kindr/common/assert_macros.hpp"

// import most common Eigen types
using namespace Eigen;

namespace dmp {

static const char* DYNAMIC_MOVEMENT_PRIMITIVE_FILE_NAME =
    "DiscreteMovementPrimitive";

/*DiscreteMovementPrimitive::DiscreteMovementPrimitive(ros::NodeHandle& node_handle) :
 initialized_(false), item_id_(-1), node_handle_(node_handle), params_(node_handle_), dmp_debug_(node_handle_)
 {
 }*/

DiscreteMovementPrimitive::DiscreteMovementPrimitive()
    : initialized_(false),
      params_() {
}

DiscreteMovementPrimitive::~DiscreteMovementPrimitive() {
  transformation_systems_.clear();
}

bool DiscreteMovementPrimitive::initialize(int num_transformation_systems,
                                           const Eigen::VectorXd num_rfs,
                                           const double activation,
                                           const bool exponentially_spaced,
                                           const double can_sys_cutoff,
                                           const double sampling_frequency,
                                           const double teaching_duration,
                                           const double execution_duration,
                                           const double alpha_z,
                                           const double beta_z) {
  DmpParameters dmp_params;
  if (!dmp_params.initialize(sampling_frequency, teaching_duration, execution_duration,
                             can_sys_cutoff, alpha_z, beta_z)) {
    printf("Could not initialize dmp parameters\n");
    initialized_ = false;
    return initialized_;
  }

  // initialize directory name and id in the base class to generate item_name_
  if (!initializeBase(0, DYNAMIC_MOVEMENT_PRIMITIVE_FILE_NAME))  // TODO
                      {
    printf("Could not initialize base.\n");
    initialized_ = false;
    return initialized_;
  }

  // assign dmp parameters
  params_ = dmp_params;

  // overwrite number of transformation system in dmp_params
  if (num_transformation_systems <= 0) {
    printf("Number of transformation system %i is not valid\n",
           num_transformation_systems);
    initialized_ = false;
    return initialized_;
  }
  params_.num_transformation_systems_ = num_transformation_systems;

  // initialized transformation systems using the gaussian kernel model parameters
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    printf("Initializing transformation system %i.\n", i);
    transformation_systems_.push_back(new TransformationSystem(this));
    if (!transformation_systems_[i].initialize(num_rfs(i), activation,
                                               exponentially_spaced,
                                               can_sys_cutoff)) {
      printf("Could not initialize transformation system %i.\n", i);
      initialized_ = false;
      return initialized_;
    }
  }

  // set the version of the dmp formulation
  //params_.version_ = version;

  // set canonical system to pre-defined state
  resetCanonicalState();

  // allocate some memory
  initialize();

  initialized_ = true;
  return initialized_;
}

void DiscreteMovementPrimitive::initialize() {
  trajectory_target_function_input_.clear();
  //debug_trajectory_point_ = VectorXd::Zero(NUM_DEBUG_CANONICAL_SYSTEM_VALUES + (params_.num_transformation_systems_ * NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES));
}

bool DiscreteMovementPrimitive::reInitializeParams() {
  //return params_.initialize();
  return true;
}

bool DiscreteMovementPrimitive::learnFromThetas(
    const std::vector<VectorXd>& thetas, const VectorXd &initial_start,
    const VectorXd &initial_goal, const double sampling_frequency,
    const double initial_duration) {
  if (!initialized_) {
    printf("DMP is not initialized.\n");
    params_.is_learned_ = false;
    return params_.is_learned_;
  }

  if (params_.num_transformation_systems_ != initial_start.size()) {
    printf(
        "Number of transformation system (%i) does not correspond to initial start dimension (%i).\n",
        params_.num_transformation_systems_, (int) initial_start.size());
    params_.is_learned_ = false;
    return params_.is_learned_;
  }

  if (params_.num_transformation_systems_ != initial_goal.size()) {
    printf(
        "Number of transformation system (%i) does not correspond to initial goal dimension (%i).\n",
        params_.num_transformation_systems_, (int) initial_goal.size());
    params_.is_learned_ = false;
    return params_.is_learned_;
  }

  // set y0 to start state of trajectory and set goal to end of the trajectory
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    // set initial start and initial goal
    transformation_systems_[i].setInitialStart(initial_start(i));
    transformation_systems_[i].setInitialGoal(initial_goal(i));
  }

  params_.alpha_x_ = -log(params_.can_sys_cutoff_);
  params_.teaching_duration_ = initial_duration;

  params_.delta_t_ = static_cast<double>(1.0) / sampling_frequency;
  params_.initial_delta_t_ = params_.delta_t_;

  params_.tau_ = params_.teaching_duration_;
  params_.initial_tau_ = params_.tau_;

  if (!setThetas(thetas)) {
    printf("Could not set theta parameters.\n");
    params_.is_learned_ = false;
    return params_.is_learned_;
  }

  printf("DMP learned from Thetas.\n");
  params_.is_learned_ = true;
  return params_.is_learned_;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::getInitialStart(VectorXd &initial_start) {
  if (initial_start.size() != params_.num_transformation_systems_) {
    return false;
  }
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    initial_start(i) = transformation_systems_[i].initial_y0_;
  }
  return true;
}
// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::getInitialGoal(VectorXd &initial_goal) {
  if (initial_goal.size() != params_.num_transformation_systems_) {
    return false;
  }
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    initial_goal(i) = transformation_systems_[i].initial_goal_;
  }
  return true;
}
// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::getGoal(VectorXd &goal) {
  if (goal.size() != params_.num_transformation_systems_) {
    return false;
  }
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    goal(i) = transformation_systems_[i].goal_;
  }
  return true;
}

bool DiscreteMovementPrimitive::setup(const double sampling_frequency) {
  VectorXd start = VectorXd::Zero(params_.num_transformation_systems_);
  VectorXd goal = VectorXd::Zero(params_.num_transformation_systems_);
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    start(i) = transformation_systems_[i].initial_y0_;
    goal(i) = transformation_systems_[i].initial_goal_;
  }

  return setup(start, goal, params_.initial_tau_, sampling_frequency);
}

bool DiscreteMovementPrimitive::setup(const VectorXd &goal,
                                      const double movement_duration,
                                      const double sampling_frequency) {
  VectorXd start = VectorXd(params_.num_transformation_systems_);
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    start(i) = transformation_systems_[i].initial_y0_;
  }
  if (!setup(start, goal, movement_duration, sampling_frequency)) {
    params_.is_setup_ = false;
    return params_.is_setup_;
  }

  // start has not been specified, need to be set before the DMP can be propagated
  params_.is_start_set_ = false;

  params_.is_setup_ = true;
  return params_.is_setup_;
}

bool DiscreteMovementPrimitive::setup(const VectorXd &start,
                                      const VectorXd &goal,
                                      const double movement_duration,
                                      const double sampling_frequency) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  if (!params_.is_learned_) {
    printf("DMP unit is not learned.\n");
    params_.is_setup_ = false;
    return params_.is_setup_;
  }

  if (start.size() != params_.num_transformation_systems_) {
    printf("Cannot set start of the DMP, the size is %i, but should be %i.",
           (int) start.size(), params_.num_transformation_systems_);
    params_.is_setup_ = false;
    return params_.is_setup_;
  }

  if (goal.size() != params_.num_transformation_systems_) {
    printf("Cannot set goal of the DMP, the size is %i, but should be %i.",
           (int) goal.size(), params_.num_transformation_systems_);
    params_.is_setup_ = false;
    return params_.is_setup_;
  }

  // reset canonical system
  resetCanonicalState();

  if (movement_duration > 0) {
    params_.tau_ = movement_duration;

    if (sampling_frequency <= 0) {
      printf("Sampling frequency %f [Hz] of the trajectory is not valid.",
             sampling_frequency);
      params_.is_setup_ = false;
      return params_.is_setup_;
    }
    params_.delta_t_ = static_cast<double>(1.0)
        / static_cast<double>(sampling_frequency);
  } else {
    params_.tau_ = params_.initial_tau_;
    params_.delta_t_ = params_.initial_delta_t_;
  }

  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    // set internal variables to zero
    transformation_systems_[i].reset();

    // set start and goal
    transformation_systems_[i].setStart(start(i));
    transformation_systems_[i].setGoal(goal(i));

    // set current state to start state (position and velocity)
    transformation_systems_[i].setState(start(i), 0.0);
  }

  // reset the sample counter
  params_.sample_index_ = 0;

  // start is set
  params_.is_start_set_ = true;

  params_.is_setup_ = true;
  return params_.is_setup_;
}

bool DiscreteMovementPrimitive::getCurrentPosition(VectorXd &current_position) {
  if (!params_.is_setup_) {
    return false;
  }
  if (current_position.size() != params_.num_transformation_systems_) {
    printf(
        "Provided vector has wrong size (%i), required size is (%i). Cannot get current position.\n",
        (int) current_position.size(), params_.num_transformation_systems_);
    return false;
  }
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    current_position(i) = transformation_systems_[i].y_;
  }
  return true;
}

bool DiscreteMovementPrimitive::getCurrentVelocity(VectorXd &current_velocity) {
  if (!params_.is_setup_) {
    return false;
  }
  if (current_velocity.size() != params_.num_transformation_systems_) {
    printf(
        "Provided vector has wrong size (%i), required size is (%i). Cannot get current velocity.\n",
        (int) current_velocity.size(), params_.num_transformation_systems_);
    return false;
  }
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    current_velocity(i) = transformation_systems_[i].yd_;
  }
  return true;
}

bool DiscreteMovementPrimitive::getCurrentAcceleration(
    VectorXd &current_acceleration) {
  if (!params_.is_setup_) {
    return false;
  }
  if (current_acceleration.size() != params_.num_transformation_systems_) {
    printf(
        "Provided vector has wrong size (%i), required size is (%i). Cannot get current acceleration.\n",
        (int) current_acceleration.size(), params_.num_transformation_systems_);
    return false;
  }
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    current_acceleration(i) = transformation_systems_[i].ydd_;
  }
  return true;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::changeGoal(const VectorXd &goal,
                                           const int start_index,
                                           const int end_index) {
  if ((!params_.is_setup_) || (start_index < 0)
      || (end_index > params_.num_transformation_systems_)
      || (end_index <= start_index)) {
    return false;
  }
  if (goal.size() != end_index - start_index) {
    printf(
        "Provided vector has wrong size (%i), required size is (%i). Cannot change goal position.\n",
        (int) goal.size(), end_index - start_index);
    return false;
  }
  for (int i = start_index; i < end_index; i++) {
    transformation_systems_[i].setGoal(goal(i - start_index));
  }
  return true;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::changeGoal(const double new_goal,
                                           const int index) {
  if ((!params_.is_setup_) || (index < 0)
      || (index > params_.num_transformation_systems_)) {
    return false;
  }
  transformation_systems_[index].setGoal(new_goal);
  return true;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::changeStart(const VectorXd &start) {
  if (!params_.is_setup_) {
    printf("DMP is not setup\n");
    return false;
  }
  if (start.size() != params_.num_transformation_systems_) {
    printf("Start vector has wrong size (%i), it should be %i.\n",
           (int) start.size(), params_.num_transformation_systems_);
    return false;
  }
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    transformation_systems_[i].setStart(start(i));
    // set current state to start state (position and velocity)
    transformation_systems_[i].setState(start(i), 0.0);
  }
  params_.is_start_set_ = true;
  return true;
}

bool DiscreteMovementPrimitive::getThetas(std::vector<VectorXd>& thetas) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  thetas.clear();
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    int num_rfs;
    if (!transformation_systems_[i].gaussianKernelModel_->getNumRFS(num_rfs)) {
      printf("Could not get number of receptive fields.\n");
      return false;
    }

    VectorXd theta_vector = VectorXd::Zero(num_rfs);
    if (!transformation_systems_[i].gaussianKernelModel_->getThetas(
        theta_vector)) {
      printf("Could not retrieve thetas from transformation system %i.\n", i);
      return false;
    }
    thetas.push_back(theta_vector);
  }
  return true;
}

bool DiscreteMovementPrimitive::setThetas(const std::vector<VectorXd>& thetas) {
  if (static_cast<int>(thetas.size()) != params_.num_transformation_systems_) {
    printf(
        "Number of thetas (%i) is not equal to number of transformation systems (%i).\n",
        (int) thetas.size(), params_.num_transformation_systems_);
    return false;
  }

  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    if (!transformation_systems_[i].gaussianKernelModel_->setThetas(
        thetas[i])) {
      printf("Could not set thetas of transformation system %i.\n", i);
      return false;
    }
  }
  return true;
}

bool DiscreteMovementPrimitive::getWidthsAndCenters(
    std::vector<Eigen::VectorXd>& widths,
    std::vector<Eigen::VectorXd>& centers) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  widths.clear();
  centers.clear();

  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    int num_rfs;
    if (!transformation_systems_[i].gaussianKernelModel_->getNumRFS(num_rfs)) {
      printf("Could not get number of receptive fields.\n");
      return false;
    }

    VectorXd centers_vector = VectorXd::Zero(num_rfs);
    VectorXd widths_vector = VectorXd::Zero(num_rfs);
    if (!transformation_systems_[i].gaussianKernelModel_->getWidthsAndCenters(
        widths_vector, centers_vector)) {
      printf("Could not retrieve thetas from transformation system %i.\n", i);
      return false;
    }
    widths.push_back(widths_vector);
    centers.push_back(centers_vector);
  }
  return true;
}

bool DiscreteMovementPrimitive::getWidthsAndCenters(
    const int trans_system_index, VectorXd& widths, VectorXd& centers) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  int num_rfs;
  KINDR_ASSERT_TRUE(std::runtime_error, !getNumRFS(trans_system_index, num_rfs),
                 "Could not get number of RFS.\n");

  KINDR_ASSERT_TRUE(std::runtime_error, widths.size() == num_rfs, "");
  KINDR_ASSERT_TRUE(std::runtime_error, centers.size() == num_rfs, "");

  if (!transformation_systems_[trans_system_index].gaussianKernelModel_
      ->getWidthsAndCenters(widths, centers)) {
    printf("Could not get widths and centers of transformation system %i.\n",
           trans_system_index);
    return false;
  }
  return true;
}

bool DiscreteMovementPrimitive::getBasisFunctions(
    const int num_time_steps, std::vector<MatrixXd>& basis_functions) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  basis_functions.clear();
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    int num_rfs;
    if (!getNumRFS(i, num_rfs)) {
      return false;
    }
    MatrixXd basis_function_matrix = MatrixXd::Zero(num_time_steps, num_rfs);
    VectorXd x_input_vector = VectorXd::Zero(num_time_steps);
    double dx = static_cast<double>(1.0)
        / static_cast<double>(num_time_steps - 1);
    x_input_vector(0) = 0.0;
    for (int j = 1; j < num_time_steps; j++) {
      x_input_vector(j) = x_input_vector(j - 1) + dx;
    }
    if (!transformation_systems_[i].gaussianKernelModel_
        ->generateBasisFunctionMatrix(x_input_vector, basis_function_matrix)) {
      printf("Gaussian kernel basis function generation failed!\n");
      return false;
    }
    basis_functions.push_back(basis_function_matrix);
  }
  return true;
}

bool DiscreteMovementPrimitive::getBasisFunctionsValues(
    const double x_input, std::vector<VectorXd> &basisFunctionsValues) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  if (basisFunctionsValues.size() != params_.num_transformation_systems_) {
    printf(
        "basisFunctionsValues vector size (%i) does not match the number of transformation systems %i.\n",
        (int) basisFunctionsValues.size(), params_.num_transformation_systems_);
    return false;
  }

  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    int num_rfs;
    if (!getNumRFS(i, num_rfs)) {
      return false;
    }

    basisFunctionsValues.at(i).setZero(num_rfs);

    if (!transformation_systems_[i].gaussianKernelModel_
        ->generateBasisFunctionVector(x_input, basisFunctionsValues.at(i))) {
      printf("Gaussian kernel basis function generation failed!\n");
      return false;
    }
  }
  return true;
}

bool DiscreteMovementPrimitive::getCanonicalSystem(
    const int num_time_steps, VectorXd& can_system_vector) {
  KINDR_ASSERT_TRUE(std::runtime_error, can_system_vector.size() == num_time_steps,
                 "");
  KINDR_ASSERT_TRUE(std::runtime_error, num_time_steps > 0, "");

  double dt = params_.tau_ / static_cast<double>(num_time_steps - 1);
  double time = 0;

  can_system_vector(0) = 1;
  for (int j = 1; j < num_time_steps; j++) {
    integrateCanonicalSystem(can_system_vector(j), time);
    time += dt;
  }
  return true;
}

bool DiscreteMovementPrimitive::getNumRFS(const int trans_id, int& num_rfs) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  if ((trans_id < 0) || (trans_id >= params_.num_transformation_systems_)) {
    printf(
        "Could not get number of receptive fields, the transformation system id (%i) is invalid.\n",
        trans_id);
    return false;
  }

  return transformation_systems_[trans_id].gaussianKernelModel_->getNumRFS(
      num_rfs);
}

bool DiscreteMovementPrimitive::getNumRFS(std::vector<int>& num_rfs) {
  num_rfs.clear();
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    int tmp_num_rfs;
    if (!getNumRFS(i, tmp_num_rfs)) {
      return false;
    }
    num_rfs.push_back(tmp_num_rfs);
  }
  return true;
}

bool DiscreteMovementPrimitive::setDuration(const double movement_duration,
                                            const int sampling_frequency) {
  if (!params_.is_setup_) {
    printf("DMP need to be setup first.");
    return false;
  }

  if (sampling_frequency <= 0) {
    printf("Sampling frequency %i [Hz] is not valid.", sampling_frequency);
    return false;
  }

  if (movement_duration <= 0.09) {
    printf("Movement duration (%f) is too small.", movement_duration);
    return false;
  }

  params_.tau_ = movement_duration;
  params_.delta_t_ = static_cast<double>(1.0)
      / static_cast<double>(sampling_frequency);
  return true;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::propagateStep(
    std::vector<VectorXd> &desired_coordinates, bool &movement_finished) {
  return propagateStep(desired_coordinates, movement_finished, params_.tau_,
                       static_cast<int>(params_.tau_ / params_.delta_t_) + 1);
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::propagateStep(
    std::vector<VectorXd> &desired_coordinates, bool &movement_finished,
    const double sampling_duration, const int num_samples) {
  if ((!params_.is_learned_) || (!params_.is_setup_)
      || (!params_.is_start_set_)) {
    if (!params_.is_learned_)
      printf("DMP is not learned.\n");
    // if(!is_setup_) printf("DMP with id %i is not setup. Need to set start, goal, and duration first.", item_id_);
    if (!params_.is_setup_)
      printf(
          "DMP with is not setup. Need to set start, goal, and duration first.\n");
    movement_finished = true;
    return false;
  }

  if (desired_coordinates.size() != params_.num_transformation_systems_) {
    printf(
        "Number of desired coordinates (%i) is not correct, it should be %i.\n",
        (int) desired_coordinates.size(), params_.num_transformation_systems_);
    movement_finished = true;
    return false;
  }

  if (num_samples <= 0) {
    printf("Number of samples (%i) is not valid.\n", num_samples);
  }

  double dt_total = sampling_duration / static_cast<double>(num_samples - 1);
  double dt_threshold = static_cast<double>(1.0)
      / params_.default_sampling_frequency_;
  int num_iteration = ceil(dt_total / dt_threshold);

  //printf("sampling duration: %f, dt_total: %f, dt_threshold: %f, num_iteration: %i\n", sampling_duration, dt_total, dt_threshold, num_iteration);

  // integrate the system, make sure that all internal variables are set properly
  if (!integrate(dt_total, num_iteration)) {
    printf("Problem while integrating the dynamical system.\n");
    movement_finished = true;
    return false;
  }
  params_.sample_index_++;

  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    desired_coordinates.at(i)(_POS_) = transformation_systems_[i].y_;
    desired_coordinates.at(i)(_VEL_) = transformation_systems_[i].yd_;
    desired_coordinates.at(i)(_ACC_) = transformation_systems_[i].ydd_;
  }

  // check whether movement has finished...
  if (params_.sample_index_ >= num_samples) {
    params_.is_setup_ = false;
    params_.is_start_set_ = false;
    movement_finished = true;
    return true;
  } else {
    movement_finished = false;
  }
  return true;
}

std::string DiscreteMovementPrimitive::getInfoString() {
  std::string info("");
  std::stringstream ss;

  info.append(std::string("id: ") + ss.str());
  ss.str("");
  ss.clear();

  info.append(std::string("\n\t"));
  info.append(std::string("initialized: "));
  if (initialized_) {
    info.append(std::string("true "));
  } else {
    info.append(std::string("false"));
  }

  info.append(std::string("  learned: "));
  if (params_.is_learned_) {
    info.append(std::string("true "));
  } else {
    info.append(std::string("false"));
  }

  info.append(std::string("  setup: "));
  if (params_.is_setup_) {
    info.append(std::string("true "));
  } else {
    info.append(std::string("false"));
  }

  info.append(std::string("  is start set: "));
  if (params_.is_start_set_) {
    info.append(std::string("true "));
  } else {
    info.append(std::string("false"));
  }

  info.append(std::string("\n") + params_.getInfoString());
  for (int i = 0; i < params_.num_transformation_systems_; i++) {
    info.append(std::string("\n\t"));
    info.append(transformation_systems_[i].getInfoString());
  }

  return info;
}

bool DiscreteMovementPrimitive::integrateAndFit() {

  return false;
}

inline bool DiscreteMovementPrimitive::integrate(const double dt_total,
                                                 const int num_iteration) {
  double dt = dt_total / static_cast<double>(num_iteration);

  // canonical system
  canonical_system_.time += dt;
  integrateCanonicalSystem(canonical_system_.x, canonical_system_.time);

  // double log_x = -log(canonical_system_.x) /* * params_.tau_*/ / params_.alpha_x_;

  for (int n = 0; n < num_iteration; n++) {
    for (int i = 0; i < params_.num_transformation_systems_; i++) {
      // compute nonlinearity using Gaussian kernel model
      double prediction = 0;
      if (!transformation_systems_[i].gaussianKernelModel_->predict(
          canonical_system_.x, prediction, true)) {
        printf("Could not predict output.\n");
        return false;
      }

      transformation_systems_[i].f_ = prediction * canonical_system_.x;  // TODO: Is this correct? Compare with predict function

      /*			transformation_systems_[i].zd_ = (params_.k_gain_ * (transformation_systems_[i].goal_ - transformation_systems_[i].y_) - params_.d_gain_
       * transformation_systems_[i].z_ - params_.k_gain_ * (transformation_systems_[i].goal_ - transformation_systems_[i].y0_)
       * canonical_system_.x + params_.k_gain_ * transformation_systems_[i].f_) * (static_cast<double> (1.0) / params_.tau_);*/

      transformation_systems_[i].zd_ = (params_.alpha_z_
          * (params_.beta_z_
              * (transformation_systems_[i].goal_
                  - transformation_systems_[i].y_)
              - transformation_systems_[i].z_) + transformation_systems_[i].f_)
          * (static_cast<double>(1.0) / params_.tau_);

      transformation_systems_[i].yd_ = transformation_systems_[i].z_
          * (static_cast<double>(1.0) / params_.tau_);
      transformation_systems_[i].ydd_ = transformation_systems_[i].zd_
          * (static_cast<double>(1.0) / params_.tau_);

      transformation_systems_[i].z_ += transformation_systems_[i].zd_ * dt;  //* params_.delta_t_;
      transformation_systems_[i].y_ += transformation_systems_[i].yd_ * dt;  //* params_.delta_t_;
    }
  }

  return true;
}

inline void DiscreteMovementPrimitive::integrateCanonicalSystem(
    double& canonical_system_x, const double canonical_system_time) const {
  canonical_system_x = exp(
      -(params_.alpha_x_ / params_.tau_) * canonical_system_time);
}

bool DiscreteMovementPrimitive::isIncreasing(int transformation_system_index,
                                             bool &is_increasing) {
  if ((transformation_system_index >= 0)
      && (transformation_system_index < params_.num_transformation_systems_)
      && params_.is_learned_) {
    return (transformation_systems_[transformation_system_index].initial_y0_
        >= transformation_systems_[transformation_system_index].initial_goal_);
  }
  return false;
}

bool DiscreteMovementPrimitive::getCanonicalSystemState(
    double &canonical_system_value, double &canonical_system_time) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  canonical_system_value = canonical_system_.x;
  canonical_system_time = canonical_system_.time;
  return true;
}

double DiscreteMovementPrimitive::getProgress() const {
  return getProgress(canonical_system_.x);
}

double DiscreteMovementPrimitive::getProgress(const double &x_input) const {
  double progress;
  if (x_input < 1.0e-8) {
    progress = 1.0;
  } else if (params_.alpha_x_ > 1.0e-8) {
    progress = -log(x_input) / params_.alpha_x_;
  } else {
    progress = 0.0;
  }
  return progress;
}

}
