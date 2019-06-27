/*!
* @file 	  SmbPathFollowingController.hpp
* @author   Johannes Pankert
* @date		  03/05/2019
* @version 	1.0
* @brief    A controller that ...
*/

#pragma once

// roco_ros
#include "roco_ros/controllers/controllers.hpp"

// state and command
#include <smb_roco/RocoCommand.hpp>
#include <smb_roco/RocoState.hpp>

// smb_path_following
#include <smb_path_following_interface/SmbPathFollowingExplicitTemplateInstantiations.h>
#include <smb_path_following_interface/SmbPathFollowingInterface.h>
#include <smb_path_following_interface/SmbPathFollowingConversions.h>

// ocs2
#include <ocs2_core/logic/rules/NullLogicRules.h>

// ros
#include <ros/ros.h>
#include <nav_msgs/Path.h>

// any_measurements
#include <any_measurements/Twist.hpp>
#include <any_measurements/Pose.hpp>
#include <any_measurements_ros/any_measurements_ros.hpp>

// util
#include <message_logger/message_logger.hpp>
#include <yaml_tools/YamlNode.hpp>

namespace smb_path_following {

class SmbPathFollowingController: virtual public roco_ros::ControllerRos<smb_roco::RocoState, smb_roco::RocoCommand> {
public:
  typedef roco_ros::ControllerRos<smb_roco::RocoState, smb_roco::RocoCommand> Base;
  typedef ocs2::SystemObservation<STATE_DIM_, INPUT_DIM_> Observation;
  typedef ocs2::MPC_Interface<STATE_DIM_, INPUT_DIM_> MpcInterface;
  typedef ocs2::CostDesiredTrajectories<SmbPathFollowingInterface::scalar_t> CostDesiredTrajectories;

  //! Construct SmbPathFollowingController.
  SmbPathFollowingController();

  //! Destruct SmbPathFollowingController.
  virtual ~SmbPathFollowingController();

protected:
  ocs2::NullLogicRules nullLogicRules_;
  SmbPathFollowingInterface mmInterface_;
  std::shared_ptr<MpcInterface> mpcInterface_;

  bool planAvailable_;
  bool observationAvailable_;
  bool desiredTrajectoryAvailable_;
  int lastSequence_;

  Observation observation_;
  boost::shared_mutex observationMutex_;

  CostDesiredTrajectories costDesiredTrajectories_;
  boost::shared_mutex costDesiredTrajectoriesMutex_;

  roco::WorkerHandle mpcUpdateWorkerHandle_;

  std::mutex pathMutex_;
  nav_msgs::Path path_;
  ros::Subscriber pathSubscriber_;

  roco::WorkerHandle rosPublishingWorkerHandle_;
  ros::Publisher currentPosePublisher_;
  ros::Publisher optimalPosePublisher_;
  ros::Publisher optimalPathPublisher_;



protected:
  //! Create controller SmbPathFollowingController.
  virtual bool create(double dt);

  //! Initialize controller SmbPathFollowingController.
  virtual bool initialize(double dt);

  //! Advance controller SmbPathFollowingController.
  virtual bool advance(double dt);

  //! Reset controller SmbPathFollowingController.
  virtual bool reset(double dt);

  //! Pre-stop controller SmbPathFollowingController.
  virtual bool preStop();

  //! Stop controller SmbPathFollowingController.
  virtual bool stop();

  //! Cleanup controller SmbPathFollowingController.
  virtual bool cleanup();

  //! Swap to controller SmbPathFollowingController with state 'swap'.
  virtual bool
  swap(double dt, const roco::ControllerSwapStateInterfacePtr &swapState);

  //! Get swap state 'swapState' of controller SmbPathFollowingController.
  virtual bool getSwapState(roco::ControllerSwapStateInterfacePtr &swapState);

  //! Add shared module 'module' to controller SmbPathFollowingController.
  virtual bool addSharedModule(const roco::SharedModulePtr &module);

protected:

  void writeDesiredTrajectory(
      const nav_msgs::Path &path,
      SmbPathFollowingController::CostDesiredTrajectories &costDesiredTrajectories,
      const Observation &currentObservation);

  bool mpcUpdate(const roco::WorkerEvent &event);

  bool publishRos(const roco::WorkerEvent &event);

  void pathCallback(const nav_msgs::PathConstPtr & path);
  void adjustTimeStamps(nav_msgs::Path & path);

};
} /* namespace smb_path_following_controller */