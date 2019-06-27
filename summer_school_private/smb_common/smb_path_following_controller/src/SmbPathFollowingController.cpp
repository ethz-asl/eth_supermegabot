/*!
 * @file 	  SmbPathFollowingController.tpp
 * @author   Johannes Pankert
 * @date		  03/05/2019
 * @version 	1.0
 * @brief    A controller that ...
 */
#pragma once

#include "smb_path_following_controller/SmbPathFollowingController.hpp"

namespace smb_path_following {

SmbPathFollowingController::SmbPathFollowingController() : Base(), mmInterface_(), observation_(), costDesiredTrajectories_(0) {
  this->setName("SmbPathFollowingController");
}

SmbPathFollowingController::~SmbPathFollowingController() {}

bool SmbPathFollowingController::create(double dt) { return true; }

bool SmbPathFollowingController::initialize(double dt) {
  // initialize variables
  planAvailable_ = false;
  observationAvailable_ = false;
  desiredTrajectoryAvailable_ = false;
  lastSequence_ = -1;
  nav_msgs::Path emptyPath;
  emptyPath.header.seq = lastSequence_;
  path_ = emptyPath;
  mmInterface_.resetMpc();
  mpcInterface_ = std::make_shared<MpcInterface>(*(mmInterface_.mpcPtr_), nullLogicRules_, true);

  // subscribe to path message
  // load parameters
  std::string parameter_path = this->getParameterPath();
  MELO_INFO_STREAM("SmbJoystickController parameter_path: " << parameter_path);
  auto controlParams_ = yaml_tools::YamlNode::fromFile(parameter_path);
  if (controlParams_.isNull()) {
    MELO_ERROR("[SmbJoystickController::loadParameters]: Could not load parameter file %s!", parameter_path.c_str());
    return false;
  }

  std::string pathSubscriberTopicName = controlParams_["subscribers"]["command_base_path"]["topic"].as<std::string>();
  int pathSubscriberQueueSize = controlParams_["subscribers"]["command_base_path"]["queue_size"].as<int>();
  pathSubscriber_ = getNodeHandle().subscribe(pathSubscriberTopicName, pathSubscriberQueueSize, &SmbPathFollowingController::pathCallback,
                                              this, ros::TransportHints().tcpNoDelay());

  std::string currentPoseTopicName = controlParams_["publishers"]["current_pose"]["topic"].as<std::string>();
  currentPosePublisher_ = getNodeHandle().advertise<geometry_msgs::PoseStamped>(currentPoseTopicName, 1, false);

  std::string optimalPoseTopicName = controlParams_["publishers"]["optimal_pose"]["topic"].as<std::string>();
  optimalPosePublisher_ = getNodeHandle().advertise<geometry_msgs::PoseStamped>(optimalPoseTopicName, 1, false);

  std::string optimalPathTopicName = controlParams_["publishers"]["optimal_path"]["topic"].as<std::string>();
  optimalPathPublisher_ = getNodeHandle().advertise<nav_msgs::Path>(optimalPathTopicName, 1, false);

  // start ros publishing worker
  roco::WorkerOptions rosPublishingWorkerOptions;
  rosPublishingWorkerOptions.autostart_ = false;
  rosPublishingWorkerOptions.frequency_ = controlParams_["ros_publishing_rate"].as<double>();
  rosPublishingWorkerOptions.name_ = "ros_publishing_worker";
  rosPublishingWorkerOptions.callback_ = boost::bind(&SmbPathFollowingController::publishRos, this, _1);
  rosPublishingWorkerHandle_ = this->addWorker(rosPublishingWorkerOptions);
  this->startWorker(rosPublishingWorkerHandle_);

  // Start mpc update worker.
  roco::WorkerOptions mpcUpdateWorkerOptions;
  mpcUpdateWorkerOptions.autostart_ = false;
  mpcUpdateWorkerOptions.frequency_ = mmInterface_.mpcSettings().mpcDesiredFrequency_;
  mpcUpdateWorkerOptions.name_ = "mpc_update_worker";
  mpcUpdateWorkerOptions.callback_ = boost::bind(&SmbPathFollowingController::mpcUpdate, this, _1);
  mpcUpdateWorkerHandle_ = this->addWorker(mpcUpdateWorkerOptions);
  this->startWorker(mpcUpdateWorkerHandle_);
  return true;
}

bool SmbPathFollowingController::advance(double dt) {
  {
    boost::lock_guard<boost::shared_mutex> lockGuard(observationMutex_);
    boost::shared_lock_guard<boost::shared_mutex> lockGuardState(this->getStateMutex());
    any_measurements::Pose currentBasePose = this->getState().getBasePose();
    SmbPathFollowingConversions::writeMpcObservation(observation_, currentBasePose);
    observationAvailable_ = true;
  }

  {
    boost::shared_lock_guard<boost::shared_mutex> lockObservation(observationMutex_);
    boost::shared_lock_guard<boost::shared_mutex> lockGuardState(this->getStateMutex());

    boost::lock_guard<boost::shared_mutex> lockDesiredTrajectory(costDesiredTrajectoriesMutex_);

    auto currentSequence = path_.header.seq;
    if (currentSequence != lastSequence_) {
      {
        std::lock_guard<std::mutex> lockGuard(pathMutex_);
        writeDesiredTrajectory(path_, costDesiredTrajectories_, observation_);
      }
      mpcInterface_->setTargetTrajectories(costDesiredTrajectories_);
      desiredTrajectoryAvailable_ = true;
      lastSequence_ = currentSequence;
    }
  }

  if (!planAvailable_) {
    MELO_WARN_THROTTLE_STREAM(3, "[SmbPathFollowingController::advance] no plan available yet. Commanding zero base twist.");
    any_measurements::Twist twistCommand;
    twistCommand.time_ = any_measurements::Time(observation_.time());
    ;
    twistCommand.twist_.setZero();
    getCommand().setBaseTwistCommand(twistCommand);
    return true;
  }

  /*  MpcInterface::input_state_matrix_t feedbackMatrix;
    MpcInterface::input_vector_t feedForwardTerm;
    size_t subsystem;
    mpcInterface_.evaluateFeedbackPolicy(observation_.time(), feedForwardTerm, feedbackMatrix, subsystem);
    MpcInterface::input_vector_t controlInput = feedForwardTerm + feedbackMatrix * observation_.state();*/

  MpcInterface::input_vector_t controlInput;
  MpcInterface::state_vector_t optimalState;
  size_t subsystem;
  mpcInterface_->evaluateFeedforwardPolicy(observation_.time(), optimalState, controlInput, subsystem);

  any_measurements::Twist twistCommand;
  twistCommand.time_ = any_measurements::Time(observation_.time());
  SmbPathFollowingConversions::readMpcInput(controlInput, twistCommand.twist_);
  {
    boost::lock_guard<boost::shared_mutex> lockGuardCommand(this->getCommandMutex());
    getCommand().setBaseTwistCommand(twistCommand);
  }
  return true;
}

bool SmbPathFollowingController::reset(double dt) { return SmbPathFollowingController::initialize(dt); }

bool SmbPathFollowingController::preStop() {
  this->cancelWorker(rosPublishingWorkerHandle_, true);
  this->cancelWorker(mpcUpdateWorkerHandle_, true);
  return true;
}

bool SmbPathFollowingController::stop() { return true; }

bool SmbPathFollowingController::cleanup() { return true; }

bool SmbPathFollowingController::swap(double dt, const roco::ControllerSwapStateInterfacePtr& swapState) {
  // Call current class reset / initialize
  return this->isInitialized() ? SmbPathFollowingController::reset(dt) : SmbPathFollowingController::initialize(dt);
}

bool SmbPathFollowingController::getSwapState(roco::ControllerSwapStateInterfacePtr& swapState) {
  swapState.reset(nullptr);
  return true;
}

bool SmbPathFollowingController::addSharedModule(const roco::SharedModulePtr& module) { return false; }

bool SmbPathFollowingController::mpcUpdate(const roco::WorkerEvent& event) {
  if (!observationAvailable_) {
    MELO_INFO_THROTTLE(3, "[SmbPathFollowingController::mpcUpdate] observation not available. Skipping mpc update.");
    return true;
  }
  if (!desiredTrajectoryAvailable_) {
    MELO_INFO_THROTTLE(3, "[SmbPathFollowingController::mpcUpdate] desired trajectory not available. Skipping mpc update.");
    return true;
  }
  {
    boost::shared_lock_guard<boost::shared_mutex> lockGuard(observationMutex_);
    mpcInterface_->setCurrentObservation(observation_);
  }
  mpcInterface_->advanceMpc();

  planAvailable_ = true;
  return true;
}

void SmbPathFollowingController::writeDesiredTrajectory(const nav_msgs::Path& path,
                                                        SmbPathFollowingController::CostDesiredTrajectories& costDesiredTrajectories,
                                                        const Observation& currentObservation) {
  costDesiredTrajectories.clear();

  // set the desired robot state to the current state in case no path has been received
  if (path.poses.size() == 0) {
    costDesiredTrajectories.desiredStateTrajectory().push_back(currentObservation.state());
    costDesiredTrajectories.desiredTimeTrajectory().push_back(currentObservation.time());
    costDesiredTrajectories.desiredInputTrajectory().push_back(MpcInterface::input_vector_t::Zero());
    MELO_WARN_THROTTLE_STREAM(3, "[SmbPathFollowingController::writeDesiredTrajectory] received trajectory is empty!");

  } else {
    costDesiredTrajectories.desiredStateTrajectory().reserve(path.poses.size());
    costDesiredTrajectories.desiredInputTrajectory().reserve(path.poses.size());
    costDesiredTrajectories.desiredTimeTrajectory().reserve(path.poses.size());

    for (int i = 0; i < path.poses.size(); ++i) {
      any_measurements::Pose pose;
      any_measurements_ros::fromRos(path.poses[i], pose);
      MpcInterface::state_vector_t targetState;
      SmbPathFollowingConversions::writeMpcState(targetState, pose.pose_);

      costDesiredTrajectories.desiredStateTrajectory().push_back(targetState);
      costDesiredTrajectories.desiredInputTrajectory().push_back(MpcInterface::input_vector_t::Zero());
      costDesiredTrajectories.desiredTimeTrajectory().push_back(pose.time_.toSeconds());
    }
  }
}

void SmbPathFollowingController::pathCallback(const nav_msgs::PathConstPtr& path) {
  nav_msgs::Path pathTemp = *path;
  if (pathTemp.poses.size() > 0) {
    double duration = (pathTemp.poses.back().header.stamp - pathTemp.poses.front().header.stamp).toSec();
    if (duration < mmInterface_.timeHorizon_) {
      MELO_WARN_STREAM("[SmbPathFollowingController::pathCallback] received path duration smaller than time horizon. Skip path. "
                       << duration << "<" << mmInterface_.timeHorizon_);
      return;
    }
  }

  adjustTimeStamps(pathTemp);
  static uint pathSequence = 1;
  pathTemp.header.seq = pathSequence++;
  {
    std::lock_guard<std::mutex> lockGuard(pathMutex_);
    path_ = pathTemp;
  }
}

void SmbPathFollowingController::adjustTimeStamps(nav_msgs::Path& path) {
  auto nowSec = ros::Time::now().toSec();
  if (path.poses.size() > 0) {
    for (int i = 0; i < path.poses.size(); i++) {
      path.poses[i].header.stamp = ros::Time(nowSec + path.poses[i].header.stamp.toSec());
    }
    if (path.poses.size() == 1) {
      path.poses.push_back(path.poses[0]);
      path.poses[1].header.stamp = ros::Time(path.poses[0].header.stamp.toSec() + mmInterface_.timeHorizon_);
    }
    double duration = (path.poses.back().header.stamp - path.poses.front().header.stamp).toSec();
    MELO_INFO_THROTTLE_STREAM(5, "[DesiredBasePathModule::adjustTimeStamps] adjusted timestamps of "
                                     << path.poses.size() << " poses in path. Duration: " << duration)
  }
}

bool SmbPathFollowingController::publishRos(const roco::WorkerEvent& event) {
  if (!planAvailable_) {
    return true;
  }

  // publish current state
  any_measurements::Pose currentBasePose;
  {
    boost::shared_lock_guard<boost::shared_mutex> lockGuardState(this->getStateMutex());
    currentBasePose = getState().getBasePose();
  }
  geometry_msgs::PoseStamped currentBasePoseRos = any_measurements_ros::toRos(currentBasePose);
  currentBasePoseRos.header.frame_id = "world";
  currentPosePublisher_.publish(currentBasePoseRos);

  // publish optimal state
  any_measurements::Pose optimalPose;
  optimalPose.time_ = any_measurements_ros::fromRos(ros::Time::now());

  MpcInterface::input_vector_t controlInput;
  MpcInterface::state_vector_t optimalState;
  size_t subsystem;
  mpcInterface_->evaluateFeedforwardPolicy(optimalPose.time_.toSeconds(), optimalState, controlInput, subsystem);

  SmbPathFollowingConversions::readMpcState(optimalState, optimalPose.pose_);

  geometry_msgs::PoseStamped optimalPoseRos = any_measurements_ros::toRos(optimalPose);
  optimalPoseRos.header.frame_id = "world";
  optimalPosePublisher_.publish(optimalPoseRos);

  // publish current rollout

  const auto& timeTrajectory = mpcInterface_->getMpcTimeTrajectory();
  const auto& stateTrajectory = mpcInterface_->getMpcStateTrajectory();
  nav_msgs::Path optimalTrajectory;
  optimalTrajectory.header.frame_id = "world";
  optimalTrajectory.header.stamp - ros::Time::now();
  for (int i = 0; i < timeTrajectory.size(); i++) {
    any_measurements::Pose pose;
    pose.time_ = any_measurements::Time(timeTrajectory[i]);
    SmbPathFollowingConversions::readMpcState(stateTrajectory[i], pose.pose_);
    optimalTrajectory.poses.push_back(any_measurements_ros::toRos(pose));
    optimalTrajectory.poses.back().header.frame_id = "world";
  }
  optimalPathPublisher_.publish(optimalTrajectory);

  // command line debugging output
  MELO_INFO_THROTTLE_STREAM(3, "[SmbPathFollowingController::advance]" << std::endl
                                                                       << "current_state:" << observation_.state().transpose() << std::endl
                                                                       << "optimalState:" << optimalState.transpose() << std::endl
                                                                       << "controlInput:" << controlInput.transpose() << std::endl
                                                                       << "current time: " << observation_.time()
                                                                       << " current trajectory timespan: [" << timeTrajectory.front() << ", "
                                                                       << timeTrajectory.back() << "]");

  return true;
}

}  // namespace smb_path_following
