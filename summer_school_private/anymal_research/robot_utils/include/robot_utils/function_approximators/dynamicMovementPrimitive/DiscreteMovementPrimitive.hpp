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

#ifndef DISCRETE_MOVEMENT_PRIMITIVE_HPP_
#define DISCRETE_MOVEMENT_PRIMITIVE_HPP_

// system includes
#include <string>
#include <math.h>
#include <sstream>
#include <errno.h>
#include <stdio.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <Eigen/Eigen>

// Local includes
#include "DmpParameters.hpp"
#include "constants.hpp"
#include "TransformationSystem.hpp"
//#include <dmp_motion_generation/math_helper.h>

namespace dmp {

// forward declaration
class TransformationSystem;

// TODO: make this class a base class and derive different versions !!!

/*!
 *  \class DynamicMovementPrimitve represents the interface to learn a movement, save it to disc, load it from disc,
 *  reproduce the movement, generalize to new target, ...
 */
class DiscreteMovementPrimitive {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! constructor
   */
  DiscreteMovementPrimitive();

  /*! destructor
   */
  ~DiscreteMovementPrimitive();

  /*! Initializes the DMP. This is necessary for all future use of the DMP.
   *
   * @param num_transformation_systems (input) Number of transformation systems (dimensions) of the DMP.
   * @param dmp_id (input) ID of the DMP
   * @param dmp_parameter_namespace (input) namespace in which the DMP parameters live on the param server
   * @param lwr_parameter_namespace (input) namespace in which the LWR parameters live on the param server
   * @return true if initialization was successful, false if it failed
   */
  bool initialize(int num_transformation_systems, const Eigen::VectorXd num_rfs,
                  const double activation,
                  const bool exponentially_spaced,
                  const double can_sys_cutoff,
                  const double sampling_frequency,
                  const double teaching_duration,
                  const double execution_duration,
                  const double alpha_z,
                  const double beta_z);

  /*! Indicates whether the DMP is initialized
   *
   * @return true if DMP is initialized, false if not
   */
  bool isInitialized() const;

  /*!
   *
   * @param parameter_namespace
   * @return
   */
  bool reInitializeParams();

  //bool learnFromMinJerk(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double duration, const double delta_t);
  /*!
   * @param theta_matrix
   * @return
   */
  bool learnFromThetas(const std::vector<Eigen::VectorXd>& thetas,
                       const Eigen::VectorXd& initial_start,
                       const Eigen::VectorXd& initial_goal,
                       const double sampling_frequency,
                       const double initial_duration);

  /*!
   */
  void print();

  /*!
   */
  // TODO: change this to not use verbosity stuff...
  std::string getInfoString();

  /*!
   * @return
   */
  bool getInitialDuration(double& initial_duration);

  /*!
   * @param initial_start
   * @return
   */
  bool getInitialStart(Eigen::VectorXd& initial_start);
  /*!
   * @param initial_goal
   * @return
   */
  bool getInitialGoal(Eigen::VectorXd& initial_goal);
  /*!
   * @param goal
   * @return
   */
  bool getGoal(Eigen::VectorXd& initial_goal);

  /*!
   * Gets the parameters of each transformation system
   * @param thetas
   * @return
   */
  bool getThetas(std::vector<Eigen::VectorXd>& thetas);

  /*!
   * Sets the parameters of each transformation system
   * @param thetas
   * @return
   */
  bool setThetas(const std::vector<Eigen::VectorXd>& thetas);

  /*!
   * Gets the linearly interpolated theta value for x_input
   * This function is used for logging and visualization
   * @param x_input
   * @param interpolatedThetas
   * @return True on success, false on failure
   */
  bool getInterpolatedTheta(const double x_input,
                            std::vector<double>& interpolatedThetas);

  /*!
   * Gets the widths and centers of each transformation system
   * @param thetas
   * @return
   */
  bool getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths,
                           std::vector<Eigen::VectorXd>& centers);

  /*!
   * @param widths
   * @param centers
   * @return
   */
  bool getWidthsAndCenters(const int trans_system_index,
                           Eigen::VectorXd& widths, Eigen::VectorXd& centers);

  /*!
   * Gets the number of receptive field used by transformation system with id trans_id
   * @param trans_id
   * @param num_rfs
   * @return
   */
  bool getNumRFS(const int trans_id, int& num_rfs);

  /*!
   * Gets the number of receptive fields for each transformation system
   * @param num_rfs
   * @return
   */
  bool getNumRFS(std::vector<int>& num_rfs);

  /*!
   *
   * @param num_time_steps
   * @param basis_functions
   * @return
   */
  bool getBasisFunctions(const int num_time_steps,
                         std::vector<Eigen::MatrixXd>& basis_functions);

  /*!
   *
   * @param x_input
   * @param currentBasisFunctionsValues
   * @return
   */
  bool getBasisFunctionsValues(
      const double x_input,
      std::vector<Eigen::VectorXd> &currentBasisFunctionsValues);

  /*! set start and goal to initial start and initial goal
   * @param sampling_frequency
   * @return
   */
  bool setup(const double sampling_frequency);
  /*!
   * @param goal
   * @param sampling_frequency
   * @return
   */
  bool setup(const Eigen::VectorXd& goal, const double movement_duration = -1.0,
             const double sampling_frequency = -1.0);

  /*!
   * @param start
   * @param goal
   * @param movement_duration
   * @param sampling_frequency
   * @return
   */
  bool setup(const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
             const double movement_duration = -1.0,
             const double sampling_frequency = -1.0);

  /*! REAL-TIME REQUIREMENTS
   * @param goal
   * @param start_index
   * @param end_index
   * @return
   */
  bool changeGoal(const Eigen::VectorXd& goal, const int start_index,
                  const int end_index);
  bool changeGoal(const double new_goal, const int index);

  /*! REAL-TIME REQUIREMENTS
   *
   * @param start
   * @return
   */

  bool changeStart(const Eigen::VectorXd& start);
  /*!
   * @return
   */
  bool isSetup() const;
  bool isStartSet() const;
  void unsetStart();

  /*!
   * @param movement_duration
   * @param sampling_frequency
   * @return
   */
  bool setDuration(const double movement_duration,
                   const int sampling_frequency);

  /*! REAL-TIME REQUIREMENTS
   *  Propagates the DMP and generates an entire rollout of size num_samples. The duration of the DMP need to be
   *  set previously using one of the setup functions. The sampling duration and the number of samples specified
   *  determine the length of the trajectory and its sampling frequecy. Note, the sampling frequency of the
   *  trajectory may change.
   *
   * @param trajectory
   * @param sampling_duration
   * @param num_samples
   * @return
   */
  //bool propagateFull(Trajectory& trajectory, const double sampling_duration, const int num_samples);
  /*! REAL-TIME REQUIREMENTS
   *
   * @param desired_coordinates
   * @param finished
   * @param current_position
   * @param movement_duration
   * @return
   */
  bool propagateStep(std::vector<Eigen::VectorXd>& desired_coordinates,
                     bool& movement_finished);

  /*! REAL-TIME REQUIREMENTS
   *
   * @param desired_coordinates
   * @param movement_finished
   * @param sampling_duration
   * @param num_samples
   * @return
   */
  bool propagateStep(std::vector<Eigen::VectorXd>& desired_coordinates,
                     bool& movement_finished, const double sampling_duration,
                     const int num_samples);

  /*! writes the data trace within dmp_debug to file. This has to happen outside the run() function since otherwise would violate the real-time constraints.
   *
   */
  //bool writeDebugTrajectory();
  //bool writeDebugTrajectory(const std::string& filename);
  /*!
   * @param current_desired_position
   * @param start_index
   * @param end_index
   * @return
   */
  //bool getCurrentPosition(Eigen::VectorXd& current_desired_position, const int start_index, const int end_index);
  bool getCurrentPosition(Eigen::VectorXd& current_position);
  bool getCurrentVelocity(Eigen::VectorXd& current_velocity);
  bool getCurrentAcceleration(Eigen::VectorXd& current_acceleration);

  /*!
   * @return
   */
  double getProgress(const double &x_input) const;

  /*!
   * @return
   */
  double getProgress() const;

  /*!
   * @return
   */
  int getNumTransformationSystems() const;

  /*!
   * @param transformation_system_index
   * @param is_increasing
   * @return
   */
  bool isIncreasing(int transformation_system_index, bool& is_increasing);

  /*!
   *
   * @param canonical_system_value
   * @param canonical_system_time
   * @return
   */
  bool getCanonicalSystemState(double& canonical_system_value,
                               double& canonical_system_time);

  /*!
   * Gets the linear progress of the canonical system from 0 to 1
   *
   * @param canonical_system_value
   * @param canonical_system_progress
   * @return
   */
  //bool getCanonicalSystemProgress(const double canonical_system_value, double& canonical_system_progress);
  /*!
   * Evaluates the canonical system using its duration (tau) and alpha_x at num_time_steps time steps going from 0 to 1.
   * @param num_time_steps
   * @param can_system_vector
   * @return
   */
  bool getCanonicalSystem(const int num_time_steps,
                          Eigen::VectorXd& can_system_vector);

  /*! Reads a DMP from the directory provided by directory_name and the dmp_id
   * @param directory_name
   * @param item_id
   * @return
   */
  //bool readFromDisc(const std::string& abs_bagfile_name);
  //bool readFromDisc(const std::string& library_directory_name, const int dmp_id, const int trial_id = 0);
  /*! Writes the DMP to the directory
   * @return
   */
  //bool writeToDisc(const int trial_id = 0);
  //bool writeToDisc(const std::string& abs_bagfile_name);
  /*!
   * @return
   */
  //Parameters::Version getVersion() const;
  /*!
   *
   * @param version
   * @return
   */
  //bool getVersion(std::string& version) const;
  // ###################################################################################################
  // TODO: (MAJOR HACK!!) these functions/variables copied over from policy_library/library::LibraryItem
  // ###################################################################################################
  /*!
   *
   * @param directory_name
   * @param item_id
   * @param item_name
   * @param is_new_dmp
   * @return
   */
  //bool initializeBase(const std::string directory_name, const int item_id, const std::string item_name, const bool is_new_dmp = true);
  bool initializeBase(const int item_id, const std::string item_name,
                      const bool is_new_dmp = true);

  /*!
   */
  int getID() const;
  bool setID(const int id);

  /*!
   */
  std::string getName() const;
  std::string getFileName(const int trial_id);

  /*!
   * @param description
   */
  void setDescription(const std::string& description);
  std::string getDescription() const;

  /*!
   * @return
   */
  std::string getClassName();

 private:

  /*!
   * the structure that contains the states of the canonical system
   */
  typedef struct {
    double x;
    double time;
  } DMPCanonical;

  /*!
   */
  bool integrateAndFit();

  /*!
   *
   * @return
   */
  bool learnTransformationTarget();

  /*!
   *
   * @return returns false if the Gaussian kernel model could not come up with a prediction for various reasons, otherwise true.
   */
  bool integrate(const double dt_total, const int num_iteration);

  /*!
   * Evaluates the canonical system at the given time step (canonical_system_time) and writes it to canonical_system_x
   * @param canonical_system_x (output)
   * @param canonical_system_time (input)
   * @return
   */
  void integrateCanonicalSystem(double& canonical_system_x,
                                const double canonical_system_time) const;

  /*!
   */
  void resetCanonicalState();

  /*!
   */
  void initialize();

  bool initialized_;

  /*!
   */
  //ros::NodeHandle node_handle_;
  /*!
   * parameters that are read from file and (usually) kept the same for all DMPs
   */
  DmpParameters params_;

  /*!
   * struct that contains the variables of the canonical system (the phase variable x and time)
   */
  DMPCanonical canonical_system_;

  /*!
   * vector of num_transformation_systems_ instances of the class DMPTransformationSystem that contain relevant
   * information for each dimension of the movement system.
   */

  boost::ptr_vector<TransformationSystem> transformation_systems_;

  std::vector<double> trajectory_target_function_input_;
};

// inline functions follow
inline bool DiscreteMovementPrimitive::isInitialized() const {
  return initialized_;
}
inline bool DiscreteMovementPrimitive::isSetup() const {
  return params_.is_setup_;
}
inline bool DiscreteMovementPrimitive::isStartSet() const {
  return params_.is_start_set_;
}
inline void DiscreteMovementPrimitive::unsetStart() {
  params_.is_start_set_ = false;
}

inline void DiscreteMovementPrimitive::resetCanonicalState() {
  canonical_system_.x = 1.0;
  canonical_system_.time = 0.0;
  params_.sample_index_ = 0;
}
inline bool DiscreteMovementPrimitive::getInitialDuration(
    double& initial_duration) {
  if (!initialized_) {
    return false;
  }
  initial_duration = params_.initial_tau_;
  return true;
}
inline int DiscreteMovementPrimitive::getNumTransformationSystems() const {
  return params_.num_transformation_systems_;
}

inline std::string DiscreteMovementPrimitive::getClassName() {
  return "DynamicMovementPrimitive";
}

// #################################################################################################
// TODO: (MAJOR HACK) these functions/variables copied over from policy_library/library::LibraryItem
// #################################################################################################
//inline bool DiscreteMovementPrimitive::initializeBase(const std::string library_directory_name, const int item_id, const std::string item_name,
//const bool is_new_dmp)
inline bool DiscreteMovementPrimitive::initializeBase(
    const int item_id, const std::string item_name, const bool is_new_dmp) {
  return (initialized_ = true);
}

}

#endif /* DISCRETE_MOVEMENT_PRIMITIVE_H_ */
