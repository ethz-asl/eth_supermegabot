/*!
 * @file     SmbLowlevelController.cpp
 * @author   Tim Sandy, Koen Kraemer
 * @date     Oct, 2018
 */

// smb lowlevel controller
#include "smb_lowlevel_controller/SmbLowLevelController.hpp"

namespace smb_lowlevel_controller {

SmbLowLevelController::SmbLowLevelController(NodeHandlePtr nh)
: any_node::Node(nh),
  timeStep_(0.035),
  timePrevIteration_(std::chrono::steady_clock::now()),
  iterDurationMs_(0.0),
  updateDurationMs_(0.0),
  timeAtStart_(0.0),
  time_(0.0),
  receiveMaxLockTime_{std::chrono::microseconds{50}},
  sendMaxLockTime_{std::chrono::microseconds{50}},
  leftWheelSpeed_(0.0),
  rightWheelSpeed_(0.0),
  smbLowLevelControllerInterface_()
{
  stopUpdating_ = false;
  updateCounter_ = 01u;
}

bool SmbLowLevelController::init()
{
  timeAtStart_ = ros::Time::now().sec + ros::Time::now().nsec * 1e-9;

  try
  {
    publishMeasurements_          = param<bool>("publish_measurements", true);
    timeStep_                     = param<double>("publishing_time_step", 0.0025);
    bool commandSmb               = param<bool>("command_smb",true);

    // Initialize communication to the smb
    smbLowLevelControllerInterface_.init(port_, *nh_, commandSmb);

    auto optionsActuatorCommands = std::make_shared<cosmo_ros::SubscriberRosOptions<SmbCommandsShm>>(
      "actuator_commands", std::bind(&SmbLowLevelController::actuatorCommandsCallback, this, std::placeholders::_1), getNodeHandle());
    optionsActuatorCommands->autoSubscribe_ = false;
    optionsActuatorCommands->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
    optionsActuatorCommands->tryRosResubscribing_ = false;

    actuatorCommandsSubscriber_ = cosmo_ros::subscribeShmRos<SmbCommandsShm,
                                                       SmbCommandsRos,
                                                       smb_ros::ConversionTraits>("actuator_commands", optionsActuatorCommands);


    auto optionsActuatorReadings = std::make_shared<cosmo_ros::PublisherRosOptions>("actuator_readings", getNodeHandle());
    optionsActuatorReadings->autoPublishRos_ = false;
    optionsActuatorReadings->rosQueueSize_ = 10u;
    optionsActuatorReadings->rosLatch_ = false;
    actuatorReadingsPublisher_ = cosmo_ros::advertiseShmRos<SmbReadingsShm,
                                                      SmbReadingsRos,
                                                      smb_ros::ConversionTraits>("actuator_readings", optionsActuatorReadings);

//     // Setup signal logger
    signal_logger::setSignalLoggerRos(&getNodeHandle());
    signal_logger::SignalLoggerOptions siloOptions;
    siloOptions.updateFrequency_ = static_cast<int>(1.0/timeStep_);
    siloOptions.maxLoggingTime_ = 100.0;
    siloOptions.loggerPrefix_ = "/smb_lowlevel_controller";
//     // siloOptions.collectScriptFileName_ = loggingScriptFilename;
    signal_logger::logger->initLogger(siloOptions);
    signal_logger::add(iterDurationMs_, std::string{"LLControllerIterDurationMs"});
    signal_logger::add(updateDurationMs_, std::string{"LLControllerUpdateDurationMs"});
    signal_logger::add(time_, std::string{"LLControllerTime"});
    signal_logger::add(actuatorCommandsMissCount_, std::string{"LLActuatorCommandsMissCount"});
    signal_logger::add(leftWheelSpeed_, std::string{"leftWheelSpeed"});
    signal_logger::add(rightWheelSpeed_, std::string{"rightWheelSpeed"});
    signal_logger::logger->updateLogger();
    signal_logger::logger->startLogger();

    if (publishMeasurements_)
    {
      any_worker::WorkerOptions pubOptions;
      pubOptions.callback_ = std::bind(&SmbLowLevelController::publishWorker, this, std::placeholders::_1);
      pubOptions.defaultPriority_ = 0; // this has low priority
      pubOptions.name_ = "SmbLowLevelController::publisherWorker";
      pubOptions.timeStep_ = std::numeric_limits<double>::infinity();
      this->addWorker(pubOptions);
    }

    // Start sending sync signal
    std::string syncName = param<std::string>("sync_name", "smb_sync");
    syncMaster_ = std::unique_ptr<cosmo::SyncMaster>(new cosmo::SyncMaster(syncName, timeStep_, 10, true));
    syncMaster_->start();

    any_worker::WorkerOptions options;
    options.callback_ = std::bind(&SmbLowLevelController::updateController, this, std::placeholders::_1);
    options.defaultPriority_ = 10; // this has high priority
    options.name_ = "SmbLowLevelController::updateController";
    options.timeStep_ = std::numeric_limits<double>::infinity();
    this->addWorker(options);

    MELO_INFO_STREAM("[SmbLowLevelController]: Initialized successfully.");

  }
  catch (message_logger::log::melo_fatal& exception)
  {
    MELO_ERROR_STREAM("[SmbLowLevelController]: Caught a message logger exception: " << exception.what());
    return false;
  }
  catch (std::exception& exception)
  {
    MELO_ERROR_STREAM("[SmbLowLevelController]: Caught a standard exception: " << exception.what());
    return false;
  }
  catch (...)
  {
    MELO_ERROR_STREAM("[SmbLowLevelController]: Caught an unspecified exception.");
    return false;
  }

  return true;
}

void SmbLowLevelController::preCleanup()
{
  smbLowLevelControllerInterface_.preCleanup();

  stopUpdating_ = true;
  cvUpdate_.notify_all();
  syncMaster_->stop(true);
}

void SmbLowLevelController::cleanup()
{
  smbLowLevelControllerInterface_.cleanup();

  try
  {
    signal_logger::logger->cleanup();

    MELO_INFO_STREAM("[SmbLowLevelController]: Cleaned up successfully.");
  }
  catch (message_logger::log::melo_fatal& exception)
  {
    MELO_ERROR_STREAM("[SmbLowLevelController]: Caught a message logger exception: " << exception.what());
  }
  catch (std::exception& exception)
  {
    MELO_ERROR_STREAM("[SmbLowLevelController]: Caught a standard exception: " << exception.what());
  }
  catch (...)
  {
    MELO_ERROR_STREAM("[SmbLowLevelController]: Caught an unspecified exception.");
  }
}

bool SmbLowLevelController::updateController(const any_worker::WorkerEvent& event)
{
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  int64_t elapsedSub;

  const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
  const int64_t maxComputationTimeNSecs = timeStepNSecs*10;

  while(!stopUpdating_)
  {
    // Waiting for sync msg from simulation or lowlevel ctrl
    auto msg = syncMaster_->waitForSync();

  //   // Stop immediately if notified.
    if (stopUpdating_) {
      return true;
    }

    auto timeCurrIteration = std::chrono::steady_clock::now();
    const int64_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(timeCurrIteration - timePrevIteration_).count();
    iterDurationMs_ = (double)elapsed*1e-6;
    timePrevIteration_ = timeCurrIteration;

    //-- Start measuring computation time.
    time_ = ros::Time::now().sec + ros::Time::now().nsec * 1e-9 - timeAtStart_;
    start = std::chrono::steady_clock::now();

    try
    {
      updateMeasurements();
      updateCommands();
      // Collect data for logger
      signal_logger::logger->collectLoggerData();
      signal_logger::logger->publishData();

      // Notify the publish worker
      updateCounter_++;
      cvUpdate_.notify_all();

    }
    catch (message_logger::log::melo_fatal& exception)
    {
      MELO_ERROR_STREAM("[SmbLowLevelController]: Caught a message logger exception: " << exception.what());
      return false;
    }
    catch (std::exception& exception)
    {
      MELO_ERROR_STREAM("[SmbLowLevelController]: Caught a standard exception: " << exception.what());
      return false;
    }
    catch (...)
    {
      MELO_ERROR_STREAM("[SmbLowLevelController]: Caught an unspecified exception.");
      return false;
    }

    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    const int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    updateDurationMs_ = (double)elapsedTimeNSecs*1e-6;
//    MELO_WARN_THROTTLE(2.0, "Computation of low-level took: %lf ms. (Warning is throttled: 1s)", (double)elapsedTimeNSecs*1e-6)
    if (elapsedTimeNSecs > timeStepNSecs) {
      MELO_WARN_THROTTLE(1.0, "Computation of low-level controller is not real-time! Computation took: %lf ms. (Warning is throttled: 1s)", (double)elapsedTimeNSecs*1e-6);
    }
  }
  return true;
}

void SmbLowLevelController::updateCommands()
{
  if(!actuatorCommandsSubscriber_->receive(receiveMaxLockTime_)) {
    ++actuatorCommandsMissCount_;
  }

  {
    boost::unique_lock<boost::mutex> lock{commandsMutex_};

    if (!smbLowLevelControllerInterface_.updateCommands(smbCommands_)) {
      MELO_WARN_STREAM("[SmbLowLevelController] Update Smb commands returned false."); //TODO take some appropriate action
    }
  }


  std::chrono::time_point<std::chrono::steady_clock> startSub, endSub;
  int64_t elapsedSub;
  double durationSubinterval;
}

void SmbLowLevelController::updateMeasurements()
{
  boost::unique_lock<boost::mutex> lock{readingsMutex_};

  std::chrono::time_point<std::chrono::steady_clock> startSub, endSub;
  int64_t elapsedSub;
  double durationSubinterval;

  if (!smbLowLevelControllerInterface_.updateMeasurements(smbReadings_)) {
    MELO_WARN_STREAM("[SmbLowLevelController] Update Smb measurements returned false."); //TODO take some appropriate action
  }

  if(!actuatorReadingsPublisher_->publish(smbReadings_, sendMaxLockTime_)) {
    ++actuatorReadingsMissCount_;
  }
}

void SmbLowLevelController::actuatorCommandsCallback(const SmbCommandsShm& msg) {
  boost::unique_lock<boost::mutex> lock{commandsMutex_};
  smbCommands_ = msg;
}

bool SmbLowLevelController::publishWorker(const any_worker::WorkerEvent& workerEvent)
{
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  unsigned long localCounter = 0lu;

  while(!stopUpdating_)
  {
    /* Suspend the thread until new quadruped state message has arrived.
    * We use the conditional variable for that, but we need to check the counter
    * to handle spurious wake-ups.
    */
    boost::unique_lock<boost::mutex> lock{mutexPublishUpdate_};
    cvUpdate_.wait(lock,[this,localCounter](){
           if (stopUpdating_) return true;
           return (updateCounter_ > localCounter);
    });
    localCounter = updateCounter_;

    // Stop immediately
    if (stopUpdating_) {
      return true;
    }

  //   //-- Start measuring computation time.
    start = std::chrono::steady_clock::now();
    actuatorReadingsPublisher_->sendRos();
  }

  return true;
}

bool SmbLowLevelController::signalLoggerWorker(const any_worker::WorkerEvent& event) {
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  unsigned long localCounter = 0lu;

  while(!stopUpdating_)
  {
    /* Suspend the thread until new quadruped state message has arrived.
    * We use the conditional variable for that, but we need to check the counter
    * to handle spurious wake-ups.
    */
    boost::unique_lock<boost::mutex> lock{mutexSignalLoggerUpdate_};
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

    // Dot the work
    signal_logger::logger->publishData();

    //-- Measure computation time.
    end = std::chrono::steady_clock::now();
    int64_t elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    const int64_t timeStepNSecs = (int64_t)(timeStep_*1e9);
    const int64_t maxComputationTimeNSecs = timeStepNSecs*10;

    if (elapsedTimeNSecs > timeStepNSecs) {
       MELO_WARN("Computation of signal logger of Low-level Controller is not real-time! Computation time: %lf ms", (double)elapsedTimeNSecs*1e-6);
    }
  }

  return true;
}

} // smb_lowlevel_controller
