/*!
 * @file     SmbLowlevelController.hpp
 * @author   Tim Sandy, Koen Kraemer
 * @date     Oct, 2018
 */

#pragma once

// any node
#include <any_node/Node.hpp>

// cosmo
#include <cosmo/SyncMaster.hpp>
#include <cosmo_ros/cosmo_ros.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// smb common
#include <smb_common/SmbCommands.hpp>
#include <smb_common/SmbReadings.hpp>

#include <smb_description/SmbDescription.hpp>
#include <smb_ros/ConvertRosMessages.hpp>

// smb lowlevel controller
#include <smb_lowlevel_controller/SmbLowLevelControllerInterface.hpp>

namespace smb_lowlevel_controller {

std::string port_ = "/dev/ttySMB"; //Make this easily configurable?

//todo How to accept an external command to change the Smb command mode?

class SmbLowLevelController : public any_node::Node
{
public:
  using SD = smb_description::SmbDescription;
  using ST = smb_description::SmbTopology;
  using SmbMode = smb_common::SmbMode;

  using SmbCommandsShm = smb_common::SmbCommands<smb_model::SmbModel::SmbDescription>;
  using SmbCommandsRos = smb_msgs::SmbCommands;

  using SmbReadingsShm = smb_common::SmbReadings<smb_model::SmbModel::SmbDescription>;
  using SmbReadingsRos = smb_msgs::SmbReadings;

  SmbLowLevelController(NodeHandlePtr nh);
  virtual ~SmbLowLevelController() = default;

  bool init();
  void preCleanup();
  void cleanup();

protected:
  void updateCommands();
  void updateMeasurements();

  void actuatorCommandsCallback(const SmbCommandsShm& msg);

  bool updateController(const any_worker::WorkerEvent& event);
  bool publishWorker(const any_worker::WorkerEvent& workerEvent);
  bool signalLoggerWorker(const any_worker::WorkerEvent& event);

protected:
  bool publishMeasurements_ = true;
  double timeStep_;

  SmbLowLevelControllerInterface<smb_model::SmbModel::RD > smbLowLevelControllerInterface_;

  SmbCommandsShm smbCommands_;
  SmbReadingsShm smbReadings_;

  double leftWheelSpeed_;
  double rightWheelSpeed_;

  unsigned int actuatorCommandsMissCount_ = 0u;
  cosmo_ros::SubscriberRosPtr<SmbCommandsShm,
                              SmbCommandsRos,
                              smb_ros::ConversionTraits> actuatorCommandsSubscriber_;
  
  unsigned int actuatorReadingsMissCount_ = 0u;
  cosmo_ros::PublisherRosPtr<SmbReadingsShm,
                             SmbReadingsRos,
                             smb_ros::ConversionTraits> actuatorReadingsPublisher_;

  std::unique_ptr<cosmo::SyncMaster> syncMaster_;


  std::chrono::time_point<std::chrono::steady_clock> timePrevIteration_;
  double iterDurationMs_;
  double updateDurationMs_;
  double timeAtStart_;
  double time_;

  boost::condition_variable_any cvUpdate_;
  boost::mutex mutexSignalLoggerUpdate_;
  boost::mutex mutexPublishUpdate_;
  boost::mutex readingsMutex_;
  boost::mutex commandsMutex_;
  boost::atomic<bool> stopUpdating_;
  std::atomic<unsigned long> updateCounter_;

  const std::chrono::microseconds receiveMaxLockTime_;
  const std::chrono::microseconds sendMaxLockTime_;
};

} /* smb_lowlevel_controller */
