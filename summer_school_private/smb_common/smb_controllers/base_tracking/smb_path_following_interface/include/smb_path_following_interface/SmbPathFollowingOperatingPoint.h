//
// Created by johannes on 29.04.19.
//

#pragma once

#include <smb_path_following_interface/SmbPathFollowingExplicitTemplateInstantiations.h>
#include "smb_path_following_interface/SmbPathFollowingDefinitions.h"

namespace smb_path_following {

class SmbPathFollowingOperatingPoint : public ocs2::SystemOperatingPoint<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<SmbPathFollowingOperatingPoint> Ptr;
  typedef std::shared_ptr<const SmbPathFollowingOperatingPoint> ConstPtr;

  typedef ocs2::SystemOperatingPoint<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_> BASE;
  typedef typename Base::scalar_t scalar_t;
  typedef typename Base::scalar_array_t scalar_array_t;
  typedef typename Base::size_array_t size_array_t;
  typedef typename Base::state_vector_t state_vector_t;
  typedef typename Base::state_vector_array_t state_vector_array_t;
  typedef typename Base::input_vector_t input_vector_t;
  typedef typename Base::input_vector_array_t input_vector_array_t;

  /**
   * Constructor
   *
   */
  SmbPathFollowingOperatingPoint() = default;

  /**
   * Destructor
   */
  ~SmbPathFollowingOperatingPoint() = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  SmbPathFollowingOperatingPoint* clone() const final { return new SmbPathFollowingOperatingPoint(*this); }

  /**
   * Gets the operating points for the system in time interval [startTime, finalTime] where there is
   * no intermediate switches except possibly the end time.
   *
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [out] timeTrajectory: Output time stamp trajectory.
   * @param [out] stateTrajectory: Output state trajectory.
   * @param [out] inputTrajectory: Output control input trajectory.
   * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or
   * override (default).
   */
  void getSystemOperatingTrajectories(const state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime,
                                      scalar_array_t& timeTrajectory, state_vector_array_t& stateTrajectory,
                                      input_vector_array_t& inputTrajectory, bool concatOutput = false) final;

 private:
};

}  // namespace smb_path_following
