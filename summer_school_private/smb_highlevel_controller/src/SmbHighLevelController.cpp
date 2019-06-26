/*!
 * @file    SmbHighLevelController.cpp
 * @author  Tim Sandy
 * @date    Oct, 2018
 */

#include <signal_logger/signal_logger.hpp>
#include <smb_highlevel_controller/SmbHighLevelController.hpp>

namespace smb_highlevel_controller {

SmbHighLevelController::SmbHighLevelController(any_node::Node::NodeHandlePtr nh)
    : any_node::Node(nh),
      isRealRobot_(true),
      timeStep_(0.02),
      mutexCommand_(new boost::shared_mutex()),
      mutexState_(new boost::shared_mutex()),
      state_(new smb_roco::RocoState()),
      command_(new smb_roco::RocoCommand()),
      controllerManager_("smb_roco::RocoState", "smb_roco::RocoCommand"),
      receiveMaxLockTime_{std::chrono::microseconds{50}},
      sendMaxLockTime_{std::chrono::microseconds{50}} {
  stopUpdating_ = false;
  // updateCounter_ = 01u;
}

bool SmbHighLevelController::init() {
  readParameters();
  ROS_INFO_STREAM("[SmbHighLevelController] Is controlling a real robot: " << (isRealRobot_ ? "yes" : "no"));

  //--- Configure logger.
  std::string loggingScriptFilename;
  getNodeHandle().param<std::string>("logger/config_file", loggingScriptFilename, "");
  if (loggingScriptFilename.empty()) {
    loggingScriptFilename = ros::package::getPath("smb_highlevel_controller") + std::string{"/config/logger.yaml"};
  }
  double loggerSamplingWindow = param<double>("logger/sampling_window", 60.0);
  std::string loggerClass;
  getNodeHandle().param<std::string>("logger/class", loggerClass, "ros");
  if (loggerClass.compare("ros") == 0) {
    MELO_INFO("[SmbHighLevelController::init] Logger type: ros");
    // initialize ros logger
    signal_logger::setSignalLoggerRos(&getNodeHandle());
  } else if (loggerClass.compare("std") == 0) {
    MELO_INFO("[SmbHighLevelController::init] Logger type: std");
    // initialize std logger as fallback logger
    signal_logger::setSignalLoggerStd();
  } else {
    MELO_INFO("[SmbHighLevelController::init] Logger type: none");
    signal_logger::setSignalLoggerNone();
  }

  signal_logger::SignalLoggerOptions options;
  options.maxLoggingTime_ = loggerSamplingWindow;
  options.updateFrequency_ = static_cast<int>(1.0 / timeStep_);
  options.collectScriptFileName_ = loggingScriptFilename;
  signal_logger::logger->initLogger(options);

  //---
  initializePublishers();
  initializeSubscribers();
  initializeControllerManager();

  std::string syncName = param<std::string>("sync_name", "smb_controller_sync");
  MELO_INFO_STREAM("Sync signal name: " << syncName);
  syncSlave_.reset(new cosmo::SyncSlave(syncName));
  syncSlave_->start();

  MELO_INFO("Start worker...");

  // Update worker
  any_worker::WorkerOptions workerOptions;
  workerOptions.callback_ = std::bind(&SmbHighLevelController::updateController, this, std::placeholders::_1);
  workerOptions.defaultPriority_ = 10;  // this has high priority
  workerOptions.name_ = "SmbHighLevelController::updateController";
  stopUpdating_ = false;
  workerOptions.timeStep_ = std::numeric_limits<double>::infinity();
  this->addWorker(workerOptions);

  return true;
}

void SmbHighLevelController::preCleanup() {
  stopUpdating_ = true;
  syncSlave_->stop(true);
}

void SmbHighLevelController::cleanup() {
  // // Shutdown subscriber.
  softEmergencyStopSubscriber_.shutdown();
  hardEmergencyStopSubscriber_.shutdown();

  // Cleanup controller manager.
  controllerManager_.cleanup();
}

void SmbHighLevelController::actuatorReadingsCallback(const SmbReadingsShm& msg) {
  // todo We don't need to do anything here for now

  //  smb_model::JointTorques jointTorques;
  //  jointTorques.setZero();
  //  for (const auto& key : KD::getKeys<KT::SmbActuatorEnum>()) {
  //    const auto actuatorEnum = key.getEnum();
  //    const auto actuatorId = key.getId();
  //    jointTorques(actuatorId) = msg.smbActuatorMeasurements_[actuatorEnum].getJointTorque();
  //  }
  //
  //  smbModel_->setJointTorques(jointTorques);
  //
  //  smb_model::JointPositions jointPositions;
  //  smb_model::JointVelocities jointVelocities;
  //  jointPositions.setZero();
  //  jointVelocities.setZero();
  //  jointVelocities.setZero();
  //  for (const auto& key : KD::getKeys<KT::SmbActuatorEnum>()) {
  //    const auto actuatorEnum = key.getEnum();
  //    const auto actuatorId = key.getId();
  //    jointPositions(actuatorId) = msg.smbActuatorMeasurements_[actuatorEnum].getJointPosition();
  //    jointVelocities(actuatorId) = msg.smbActuatorMeasurements_[actuatorEnum].getJointVelocity();
  //  }
  //
  //  smb_model::SmbState newState;
  //
  //  newState.setJointPositions(jointPositions);
  //  newState.setJointVelocities(jointVelocities);
  //  newState.setJointTorques(jointTorques);
  //  newState.setPositionWorldToBaseInWorldFrame(smb_model::Position(0.0, 0.0, 0.0));
  //  newState.setOrientationBaseToWorld(
  //      smb_model::RotationQuaternion(1, 0, 0, 0));
  //
  //  smbModel_->setState(newState, true, true, false);
  //  state_->setStatus(smb_roco::RocoState::STATUS_OK);
  //
  //  if (!desiredEEStateInitialized_) {
  //    initDesiredEEState();
  //  }
}

void SmbHighLevelController::initializeControllerManager() {
  rocoma::LoggerOptions managerLoggerOptions;
  managerLoggerOptions.enable = param<bool>("logger/enable", false);
  MELO_INFO_STREAM("[MabiMobileHighlevelController::init] signal logger enabled: " << managerLoggerOptions.enable);
  managerLoggerOptions.updateOnStart = param<bool>("logger/update_on_start", false);
  std::string logFileType = param<std::string>("logger/logfile_type", "binary");
  if (logFileType == "csv") {
    managerLoggerOptions.fileTypes = {signal_logger::LogFileType::CSV};
  } else if (logFileType == "bag") {
    managerLoggerOptions.fileTypes = {signal_logger::LogFileType::BAG};
  } else {
    managerLoggerOptions.fileTypes = {signal_logger::LogFileType::BINARY};
  }

  //--- Configure controllers
  rocoma_ros::ControllerManagerRosOptions managerOptions;
  managerOptions.timeStep = timeStep_;
  managerOptions.isRealRobot = isRealRobot_;
  managerOptions.nodeHandle = this->getNodeHandle();
  managerOptions.loggerOptions = managerLoggerOptions;
  controllerManager_.init(managerOptions);
  controllerManager_.setupControllersFromParameterServer(state_, command_, mutexState_, mutexCommand_);
  controllerManager_.switchController("SmbJoystickController");
}

void SmbHighLevelController::readParameters() {
  // read some parameters
  bool simulation = param<bool>("simulation", true);
  isRealRobot_ = !simulation;
  timeStep_ = param<double>("time_step", 0.0025);
  publishRos_ = param<bool>("publish_ros", false);

  boost::lock_guard<boost::shared_mutex> commandLock(*mutexCommand_);
  command_->setDt(timeStep_);
  command_->setMaxLinearVelocity(param<double>("maxLinearVelocity", 2.0));
  command_->setMaxRotationalVelocity(param<double>("maxRotationalVelocity", 2.0));
  command_->setMaxLinearAcceleration(param<double>("maxLinearAcceleration", 5.0));
  command_->setMaxRotationalAcceleration(param<double>("maxRotationalAcceleration", 10.0));
}

bool SmbHighLevelController::updateController(const any_worker::WorkerEvent& event) {
  MELO_INFO("[SmbHighLevelController::updateController] Started updateController.");
  std::chrono::time_point<std::chrono::steady_clock> end;
  const int64_t timeStepNSecs = (int64_t)(timeStep_ * 1e9);

  while (!stopUpdating_) {
    // Waiting for sync msg from simulation or lowlevel ctrl
    auto msg = syncSlave_->waitForSync();

    // Stop immediately if notified.
    if (stopUpdating_) {
      return true;
    }

    //-- Start measuring computation time.
    t_now_ = std::chrono::steady_clock::now();

    receiveMeasurements();
    controllerManager_.updateController();
    sendCommands();

    signal_logger::logger->collectLoggerData();

    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    const int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - t_now_).count();

    if (elapsedTimeNSecs > timeStepNSecs)
      MELO_WARN_THROTTLE(3.0, "Computation of high-level controller is not real-time! Computation took: %lf ms. (Warning is throttled: 3s)",
                         (double)elapsedTimeNSecs * 1e-6);

    if (publishRos_) {
      publishRos();
      signal_logger::logger->publishData();
    }
  }
  return true;
}

bool SmbHighLevelController::publishWorker(const any_worker::WorkerEvent& event) { return true; }

void SmbHighLevelController::receiveMeasurements() {
  if (!actuatorReadingsSubscriber_->receive(receiveMaxLockTime_)) {
    ++actuatorReadingsMissCount_;
  }
  stateSubscriber_->receive(receiveMaxLockTime_);
}

void SmbHighLevelController::sendCommands() {
  SmbCommandsShm actuatorCommands;
  {
    boost::shared_lock<boost::shared_mutex> lockGuard(*mutexCommand_);
    actuatorCommands = command_->getActuatorCommands();
  }

  SmbCommandsRos rosActuatorCommands;
  smb_ros::ConvertRosMessages<smb_description::SmbDescription>::writeToMessage(rosActuatorCommands, actuatorCommands);
  // Send
  if (!actuatorCommandsPublisher_->publish(actuatorCommands, rosActuatorCommands, sendMaxLockTime_)) {
    MELO_WARN_THROTTLE(1.0, "[SmbHighLevelController] Publishing actuator commands failed!");
    ++actuatorCommandsMissCount_;
  }
}

void SmbHighLevelController::initializePublishers() {
  auto acOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("actuator_commands", getNodeHandle());
  acOptions->rosQueueSize_ = 10u;
  acOptions->rosLatch_ = false;
  acOptions->autoPublishRos_ = false;
  actuatorCommandsPublisher_ =
      cosmo_ros::advertiseShmRos<SmbCommandsShm, SmbCommandsRos, smb_ros::ConversionTraits>("actuator_commands", acOptions);
}

void SmbHighLevelController::initializeSubscribers() {
  std::cout << "Initializing subscribers" << std::endl;

  auto arOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<SmbReadingsShm>>(
      "actuator_readings", std::bind(&SmbHighLevelController::actuatorReadingsCallback, this, std::placeholders::_1), getNodeHandle());
  arOptions->autoSubscribe_ = false;
  arOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  arOptions->tryRosResubscribing_ = false;
  actuatorReadingsSubscriber_ =
      cosmo_ros::subscribeShmRos<SmbReadingsShm, SmbReadingsRos, smb_ros::ConversionTraits>("actuator_readings", arOptions);

  auto qsOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<SmbStateShm>>(
      "smb_state", std::bind(&SmbHighLevelController::smbStateCallback, this, std::placeholders::_1), getNodeHandle());
  qsOptions->autoSubscribe_ = false;
  qsOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  qsOptions->tryRosResubscribing_ = false;

  stateSubscriber_ = cosmo_ros::subscribeShmRos<SmbStateShm, SmbStateRos, smb_description_ros::ConversionTraits>("smb_state", qsOptions);
  ;

  softEmergencyStopSubscriber_ =
      nh_->subscribe("/soft_emcy_stop", 10, &SmbHighLevelController::softEmergencyStopCallback, this, ros::TransportHints().tcpNoDelay());
  hardEmergencyStopSubscriber_ =
      nh_->subscribe("/hard_emcy_stop", 10, &SmbHighLevelController::hardEmergencyStopCallback, this, ros::TransportHints().tcpNoDelay());
}

void SmbHighLevelController::publishRos() { actuatorCommandsPublisher_->sendRos(); }

void SmbHighLevelController::softEmergencyStopCallback(const std_msgs::Empty::ConstPtr& msg) {
  MELO_WARN("SmbHighLevelController: Received soft emergency stop.");
  controllerManager_.ControllerManager::emergencyStop();
}
void SmbHighLevelController::hardEmergencyStopCallback(const std_msgs::Empty::ConstPtr& msg) {
  MELO_WARN("SmbHighLevelController: Received hard emergency stop.");
  controllerManager_.ControllerManager::emergencyStop();
}

void SmbHighLevelController::smbStateCallback(const smb_description::SmbState& state) {
  boost::lock_guard<boost::shared_mutex> rocoStateLock(*mutexState_);
  any_measurements::Pose basePose;
  basePose.time_ = state.time_;
  basePose.pose_.getPosition() = state.smbState_.getPositionWorldToBaseInWorldFrame();
  basePose.pose_.getRotation() = state.smbState_.getOrientationBaseToWorld();
  *(state_->getBasePosePtr()) = basePose;
  if ((any_measurements_ros::fromRos(ros::Time::now()) - state.time_).toSeconds() > 0.1) {
    MELO_WARN_STREAM("[SmbHighLevelController::smbStateCallback] The received state estimate is older than 0.1. now: "
                     << any_measurements_ros::fromRos(ros::Time::now()) << "s state.time_:" << state.time_.toSeconds() << "s.")
  }
}

}  // namespace smb_highlevel_controller
