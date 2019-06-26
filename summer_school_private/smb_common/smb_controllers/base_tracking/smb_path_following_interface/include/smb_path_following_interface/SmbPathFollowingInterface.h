//
// Created by johannes on 29.04.19.
//

#pragma once

#include <smb_path_following_interface/SmbPathFollowingExplicitTemplateInstantiations.h>

// C++
#include <stdlib.h>
#include <iostream>
#include <string>

// OCS2
#include <ocs2_core/misc/loadEigenMatrix.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_slq/SLQ_Settings.h>

// MM
#include <smb_path_following_interface/SmbPathFollowingOperatingPoint.h>
#include "smb_path_following_interface/SmbPathFollowingCost.h"
#include "smb_path_following_interface/SmbPathFollowingDefinitions.h"
#include "smb_path_following_interface/SmbPathFollowingModelSettings.h"
#include "smb_path_following_interface/SmbPathFollowingSystemDynamics.h"

// ros
#include <ros/package.h>
#include <ros/ros.h>

namespace smb_path_following {

class SmbPathFollowingInterface : public ocs2::RobotInterfaceBase<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using waco_system_dynamics_t = SmbPathFollowingSystemDynamics;
  using dim_t = ocs2::Dimensions<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
  using base_cost_t = ocs2::CostFunctionBase<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
  using constraint_t = ocs2::ConstraintBase<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
  typedef ocs2::MPC_SLQ<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_> mpc_t;

  /**
   * Constructor
   */
  SmbPathFollowingInterface();

  /**
   * Destructor
   */
  ~SmbPathFollowingInterface() = default;

  /**
   * setup all optimizes.
   *
   */
  void setupOptimizer(const std::string& taskFile) final { resetMpc(); };

  /**
   * reset the mpc module
   */
  void resetMpc();

  /**
   * Gets a pointer to the internal SLQ-MPC class.
   *
   * @return Pointer to the internal MPC
   */
  mpc_t::Ptr& getMPCPtr();

  /**
   * Get the model settings.
   *
   * @return Model settings.
   */
  SmbPathFollowingModelSettings& modelSettings();

 protected:
  /**
   * Loads the settings from the path file.
   *
   * @param [in] taskFile: Task's file full path.
   */
  void loadSettings(const std::string& taskFile) final;

 public:
  std::string taskFile_;
  std::string libraryFolder_;

  mpc_t::Ptr mpcPtr_;

  SmbPathFollowingModelSettings modelSettings_;

  waco_system_dynamics_t::Ptr wacoSystemDynamicsPtr_;
  base_cost_t::Ptr costPtr_;
  SmbPathFollowingOperatingPoint::Ptr operatingPointPtr_;
  constraint_t::Ptr constraintPtr_;

  size_t numPartitions_ = 0;
  /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
  scalar_t timeHorizon_ = 1;
  dim_t::scalar_array_t partitioningTimes_;
};

}  // namespace smb_path_following
