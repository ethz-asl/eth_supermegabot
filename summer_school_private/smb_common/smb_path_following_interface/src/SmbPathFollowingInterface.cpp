#include "smb_path_following_interface/SmbPathFollowingInterface.h"

namespace smb_path_following {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SmbPathFollowingInterface::SmbPathFollowingInterface() {
  std::string packagePath = ros::package::getPath("smb_path_following_interface");

  taskFile_ = packagePath + "/config/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = packagePath + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  // load setting from loading file
  loadSettings(taskFile_);

  // MPC
  resetMpc();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SmbPathFollowingInterface::loadSettings(const std::string& taskFile) {
  /*
   * Default initial condition
   */
  loadInitialState(taskFile, initialState_);

  /*
   * SLQ-MPC settings
   */
  slqSettings_.loadSettings(taskFile);
  mpcSettings_.loadSettings(taskFile);

  /*
   * Model settings
   */
  modelSettings_.loadSettings(taskFile_, true);

  /*
   * Dynamics
   */
  wacoSystemDynamicsPtr_.reset(new waco_system_dynamics_t(modelSettings_.libraryFilesAreGenerated_));

  if (modelSettings_.libraryFilesAreGenerated_ == true) {
    wacoSystemDynamicsPtr_->loadModels("mm_dynamics", libraryFolder_);
  } else {
    wacoSystemDynamicsPtr_->createModels("mm_dynamics", libraryFolder_);
  }

  /*
   * Cost function
   */

  // joint space tracking cost
  SmbPathFollowingCost::state_matrix_t jointQ_;
  SmbPathFollowingCost::input_matrix_t jointR_;
  SmbPathFollowingCost::state_matrix_t jointQFinal_;
  ocs2::loadEigenMatrix(taskFile, "joint_task.Q", jointQ_);
  ocs2::loadEigenMatrix(taskFile, "joint_task.R", jointR_);
  ocs2::loadEigenMatrix(taskFile, "joint_task.Q_final", jointQFinal_);

  std::cerr << "Q:       \n" << jointQ_ << std::endl;
  std::cerr << "R:       \n" << jointR_ << std::endl;
  std::cerr << "Q_final: \n" << jointQFinal_ << std::endl;

  SmbPathFollowingCost::Ptr baseTrackingCost(new SmbPathFollowingCost(jointQ_, jointR_, jointQFinal_, modelSettings_.libraryFilesAreGenerated_));

  if (modelSettings_.libraryFilesAreGenerated_ == true) {
    baseTrackingCost->loadModels("base_tracking_cost", libraryFolder_);
  } else {
    baseTrackingCost->createModels("base_tracking_cost", libraryFolder_);
  }

  costPtr_ = baseTrackingCost;

  constraintPtr_.reset(new constraint_t());

  operatingPointPtr_.reset(new SmbPathFollowingOperatingPoint());

  definePartitioningTimes(taskFile, timeHorizon_, numPartitions_, partitioningTimes_, true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SmbPathFollowingInterface::resetMpc() {
  mpcPtr_.reset(new mpc_t(wacoSystemDynamicsPtr_.get(), wacoSystemDynamicsPtr_.get(), constraintPtr_.get(), costPtr_.get(),
                          operatingPointPtr_.get(), partitioningTimes_, slqSettings_, mpcSettings_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SmbPathFollowingInterface::mpc_t::Ptr& SmbPathFollowingInterface::getMPCPtr() { return mpcPtr_; }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SmbPathFollowingModelSettings& SmbPathFollowingInterface::modelSettings() { return modelSettings_; }
}  // namespace smb_path_following
