/*!
 * @file    AnyStateEstimator.tpp
 * @author  Markus Staeuble
 * @date    Apr, 2018
 */

// state estimator
#include "any_state_estimator/AnyStateEstimator.hpp"

namespace any_state_estimator {

template<typename RobotContainersRos_>
AnyStateEstimator<RobotContainersRos_>::AnyStateEstimator(any_node::Node::NodeHandlePtr nh) :
    cosmo_node::Node(nh),
    timePrevIteration_(std::chrono::steady_clock::now()) {}

template<typename RobotContainersRos_>
bool AnyStateEstimator<RobotContainersRos_>::init() {
  syncSlaveName_ =  param<std::string>("sync_slave_name", "robot_sync");
  syncMasterName_ = param<std::string>("sync_master_name", "hlc_sync");
  isSimulation_ = param<bool>("simulation", false);
  isStandalone_ = param<bool>("standalone", false);
  timeStep_ = param<double>("time_step", 0.0025);

  auto robotStateOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("robot_state", getNodeHandle());
  robotStateOptions->rosQueueSize_ = 10u;
  robotStateOptions->rosLatch_ = false;
  robotStateOptions->autoPublishRos_ = false;                                                     

  robotStatePublisher_ = advertiseShmRos<RobotStateShm, RobotStateRos, RobotStateContainer::template ConversionTrait>(
    "robot_state", robotStateOptions);

  auto imuOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<ImuShm>>(
    "imu", std::bind(&AnyStateEstimator<RobotContainersRos_>::imuCallback, this, std::placeholders::_1), getNodeHandle());
  imuOptions->autoSubscribe_ = false;
  imuOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  imuOptions->tryRosResubscribing_ = false;
  imuOptions->rosQueueSize_ = 10u;
  imuSubscriber_ = subscribeShmRos<ImuShm, ImuRos, ImuContainer::template ConversionTrait>("imu", imuOptions);

  auto actReadOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<ActuatorReadingsShm>>(
    "actuator_readings", std::bind(&AnyStateEstimator<RobotContainersRos_>::actuatorReadingsCallback, this, std::placeholders::_1), getNodeHandle());
  actReadOptions->autoSubscribe_ = false;
  actReadOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  actReadOptions->tryRosResubscribing_ = false;
  actReadOptions->rosQueueSize_ = 10u;
  actuatorReadingsSubscriber_ = subscribeShmRos<ActuatorReadingsShm, ActuatorReadingsRos, ActuatorReadingsContainer::template ConversionTrait>(
    "actuator_readings", actReadOptions);

  resetService_ = advertiseService(
    "reset", "reset", &AnyStateEstimator<RobotContainersRos_>::resetService, this);
  resetHereService_ = advertiseService(
    "reset_here", "reset_here", &AnyStateEstimator<RobotContainersRos_>::resetHereService, this);

  readParameters();
  initializeMessages();
  initializePublishers();
  initializeSubscribers();
  advertiseServices();
  initImpl();
  startWorkers();

  return true;
}


template<typename RobotContainersRos_>
void AnyStateEstimator<RobotContainersRos_>::startWorkers() {
  // Initialize signal logger
  signal_logger::setSignalLoggerRos(&getNodeHandle());
  signal_logger::SignalLoggerOptions siloOptions;

  siloOptions.updateFrequency_ = static_cast<int>(1.0/timeStep_);
  siloOptions.maxLoggingTime_ = 100.0;
  siloOptions.loggerPrefix_ = this->param<std::string>("logger_prefix", "/stateEstimator");
  siloOptions.loggerName_ = "stateEstimator";

  // Add variables to logger
  signal_logger::logger->initLogger(siloOptions);
  addVariablesToLog();
  signal_logger::add(iterDurationMs_, std::string{"EstimatorIterDurationMs"});
  signal_logger::add(updateDurationMs_, std::string{"EstimatorUpdateDurationMs"});
  signal_logger::logger->updateLogger();
  signal_logger::logger->startLogger();

  // Set up syncing
  syncSlave_.reset(new cosmo::SyncSlave(syncSlaveName_));
  syncMaster_.reset(new cosmo::SyncMaster(syncMasterName_, timeStep_, 10, false));

  // Start sending sync signal
  syncMaster_->start();

  stopUpdating_ = false;

  if (!isStandalone_){
    // Start receiving sync signal
    syncSlave_->start();
    // Add worker to update the state estimator in sync with the lowlevel ctrlr
    any_worker::WorkerOptions optionsEstimator;
    optionsEstimator.callback_ = std::bind(&AnyStateEstimator<RobotContainersRos_>::updateSynced, this, std::placeholders::_1);
    optionsEstimator.defaultPriority_ = 10;
    optionsEstimator.name_ = ros::this_node::getName() + "/updateSyncedWorker";
    optionsEstimator.timeStep_ = std::numeric_limits<double>::infinity();
    this->addWorker(optionsEstimator);
  }
  else {
    // Add worker to update the state estimator standalone
    any_worker::WorkerOptions optionsEstimator;
    optionsEstimator.callback_ = std::bind(
      static_cast<bool (AnyStateEstimator<RobotContainersRos_>::*)(const any_worker::WorkerEvent& event)>(&AnyStateEstimator<RobotContainersRos_>::update),
      this, std::placeholders::_1);
    optionsEstimator.defaultPriority_ = 59;
    optionsEstimator.name_ = ros::this_node::getName() + "/updateWorker";
    optionsEstimator.timeStep_ = timeStep_;
    this->addWorker(optionsEstimator);
  }

  // Add worker to publish ROS messages.
  any_worker::WorkerOptions optionsPublisher;
  optionsPublisher.callback_ = std::bind(
      static_cast<bool (AnyStateEstimator<RobotContainersRos_>::*)(const any_worker::WorkerEvent& event)>(&AnyStateEstimator<RobotContainersRos_>::publishRos),
      this, std::placeholders::_1);
  optionsPublisher.defaultPriority_ = 0; // this has low priority
  optionsPublisher.name_ = ros::this_node::getName() + "/publishRosWorker";
  optionsPublisher.timeStep_ = std::numeric_limits<double>::infinity();
  this->addWorker(optionsPublisher);
}

template<typename RobotContainersRos_>
void AnyStateEstimator<RobotContainersRos_>::preCleanup() {
  // Stop all threads
  stopUpdating_ = true;
  cvUpdate_.notify_all();

  signal_logger::logger->stopLogger();
  signal_logger::logger->saveLoggerData({signal_logger::LogFileType::CSV});

  if (!isStandalone_) {
    syncSlave_->stop(true);
  }
  syncMaster_->stop(true);
}

template<typename RobotContainersRos_>
void AnyStateEstimator<RobotContainersRos_>::cleanup() {
  signal_logger::logger->cleanup();
}


template<typename RobotContainersRos_>
void AnyStateEstimator<RobotContainersRos_>::imuCallback(const ImuShm& msg) {
  imu_ = msg;
}


template<typename RobotContainersRos_>
void AnyStateEstimator<RobotContainersRos_>::update() {
  std::lock_guard<std::mutex> lockStateEstimator(mutexStateEstimator_);
  const auto timeNow = any_measurements_ros::fromRos(ros::Time::now());
  receiveMeasurements();
  preprocessMeasurements();
  advanceEstimator();
  setOutput();
  publish();

  syncMaster_->sync(timeNow, timeStep_);

  // collect the data
  signal_logger::logger->collectLoggerData();
}

template<typename RobotContainersRos_>
bool AnyStateEstimator<RobotContainersRos_>::update(const any_worker::WorkerEvent& event) {
  std::chrono::time_point<std::chrono::steady_clock> start, end;

  if(!stopUpdating_) {

    auto timeCurrIteration = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(timeCurrIteration - timePrevIteration_).count();
    iterDurationMs_ = static_cast<double>(elapsed*1e-6);
    timePrevIteration_ = timeCurrIteration;

    start = std::chrono::steady_clock::now();

    // Do the work now.
    update();
    // Notify publisher worker
    updateCounter_++;
    cvUpdate_.notify_all();

    end = std::chrono::steady_clock::now();
    
    const auto elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    updateDurationMs_ = static_cast<double>(elapsedTimeNSecs*1e-6);
  }
  return true;
}

template<typename RobotContainersRos_>
bool AnyStateEstimator<RobotContainersRos_>::updateSynced(const any_worker::WorkerEvent& event) {
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  const auto timeStepNSecs = static_cast<int64_t>(timeStep_*1e9);

  while(!stopUpdating_)
  {

    auto msg = syncSlave_->waitForSync();

    // Stop immediately if notified.
    if (stopUpdating_) {
      return true;
    }

    auto timeCurrIteration = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(timeCurrIteration - timePrevIteration_).count();
    iterDurationMs_ = static_cast<double>(elapsed*1e-6);
    // MELO_INFO_STREAM("[State Estimator]: Time between two iterations: " << iterDurationMs_);
    timePrevIteration_ = timeCurrIteration;

    //-- Start measuring computation time.
    start = std::chrono::steady_clock::now();

    // Do the work now.
    update();

    // Notify publisher worker
    updateCounter_++;
    cvUpdate_.notify_all();

    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    const auto elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    updateDurationMs_ = static_cast<double>(elapsedTimeNSecs*1e-6);
    if (elapsedTimeNSecs > timeStepNSecs) {
      MELO_WARN_THROTTLE(3.0, 
        "Computation of state estimator is not real-time! Computation took: %lf ms. (Warning is throttled: 3s)", static_cast<double>(elapsedTimeNSecs*1e-6));
    }

  }

  return true;
}

template<typename RobotContainersRos_>
bool AnyStateEstimator<RobotContainersRos_>::publishRos( const any_worker::WorkerEvent& event) {

  std::chrono::time_point<std::chrono::steady_clock> start, end;
  unsigned long localCounter = 0lu;

  while(!stopUpdating_)
  {
    /* Suspend the thread until new robot state message has arrived.
    * We use the conditional variable for that, but we need to check the counter
    * to handle spurious wake-ups.
    */
    std::unique_lock<std::mutex> lock{mutexUpdate_};
    cvUpdate_.wait(lock,[this,localCounter](){
           if (stopUpdating_) return true;
           return (updateCounter_ > localCounter);
    });
    localCounter = updateCounter_;

    // Stop immediately
    if (stopUpdating_) {
      return true;
    }

    //-- Start measuring computation time.
    start = std::chrono::steady_clock::now();

    // Publish over ROS
    publishRos();

    // Publish signal logger data.
    signal_logger::logger->publishData();

    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    auto elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    const auto timeStepNSecs = static_cast<int64_t>(timeStep_*1e9);

    if (elapsedTimeNSecs > timeStepNSecs) {
       MELO_WARN("Computation of ROS publishers of State Estimator is not real-time! Computation time: %lf ms", 
        static_cast<double>(elapsedTimeNSecs*1e-6));
    }
  }
  return true;
}

template<typename RobotContainersRos_>
bool AnyStateEstimator<RobotContainersRos_>::resetService(any_state_estimator_msgs::ResetStateEstimator::Request& req,
                                                          any_state_estimator_msgs::ResetStateEstimator::Response& res) {
  std::lock_guard<std::mutex> lockStateEstimator(mutexStateEstimator_);
  kindr::HomTransformQuatD pose;
  kindr_ros::convertFromRosGeometryMsg(req.pose, pose);
  resetEstimator(pose);
  return true;
}

template<typename RobotContainersRos_>
bool AnyStateEstimator<RobotContainersRos_>::resetHereService(std_srvs::Trigger::Request& req,
                                                              std_srvs::Trigger::Response& res) {
  std::lock_guard<std::mutex> lockStateEstimator(mutexStateEstimator_);
  resetEstimatorHere();
  res.success = true;
  return true;
}

template<typename RobotContainersRos_>
void AnyStateEstimator<RobotContainersRos_>::actuatorReadingsCallback(const ActuatorReadingsShm& msg) {
  actuatorReadings_ = msg;
}

} /* namespace state_estimator */
