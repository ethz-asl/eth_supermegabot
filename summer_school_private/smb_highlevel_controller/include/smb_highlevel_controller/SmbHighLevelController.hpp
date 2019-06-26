/*!
 * @file    SmbHighLevelController.hpp
 * @author  Tim Sandy
 * @date    Oct, 2018
 */

#pragma once

// ros
#include <geometry_msgs/TwistStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

// any...
#include <any_node/Node.hpp>

// smb_description
#include <smb_description/Containers.hpp>
#include <smb_description/SmbDescription.hpp>
#include <smb_description_ros/smb_description_ros.hpp>

// smb_common
#include <smb_common/SmbCommands.hpp>
#include <smb_common/SmbModes.hpp>
#include <smb_common/SmbReadings.hpp>

// smb_msgs
#include <smb_msgs/SmbCommands.h>
#include <smb_msgs/SmbReadings.h>
#include <smb_msgs/SmbState.h>
#include <smb_ros/ConvertRosMessages.hpp>

// smb_model
#include <smb_model/SmbModel.hpp>

// cosmo
#include <cosmo/SyncSlave.hpp>
#include <cosmo_ros/cosmo_ros.hpp>

// roco
#include <rocoma_ros/ControllerManagerRos.hpp>
#include <smb_roco/RocoCommand.hpp>
#include <smb_roco/RocoState.hpp>
//#include <smb_roco/smb_roco.hpp>
//#include <rocoma_ros/ControllerManagerRos.hpp>

namespace smb_highlevel_controller {

class SmbHighLevelController : public any_node::Node {
 public:
  using SmbCommandsShm = smb_common::SmbCommands<smb_model::SmbModel::SmbDescription>;
  using SmbCommandsRos = smb_msgs::SmbCommands;

  using SmbReadingsShm = smb_common::SmbReadings<smb_model::SmbModel::SmbDescription>;
  using SmbReadingsRos = smb_msgs::SmbReadings;

  using SmbStateShm = smb_description::SmbState;
  using SmbStateRos = smb_msgs::SmbState;

  SmbHighLevelController(any_node::Node::NodeHandlePtr nh);

  bool init() override;
  void preCleanup() override;
  void cleanup() override;
  //  bool update(const any_worker::WorkerEvent& event) override;

 private:
  void actuatorReadingsCallback(const SmbReadingsShm& msg);
  void smbStateCallback(const SmbStateShm& state);

  bool updateController(const any_worker::WorkerEvent& event);

  bool publishWorker(const any_worker::WorkerEvent& event);

  void receiveMeasurements();
  void sendCommands();

  void initializePublishers();
  void initializeSubscribers();
  void initializeControllerManager();
  void publishRos();
  void readParameters();

  void softEmergencyStopCallback(const std_msgs::Empty::ConstPtr& msg);
  void hardEmergencyStopCallback(const std_msgs::Empty::ConstPtr& msg);

  rocoma_ros::ControllerManagerRos<smb_roco::RocoState, smb_roco::RocoCommand> controllerManager_;
  std::shared_ptr<smb_roco::RocoState> state_;
  std::shared_ptr<smb_roco::RocoCommand> command_;
  std::shared_ptr<boost::shared_mutex> mutexState_;
  std::shared_ptr<boost::shared_mutex> mutexCommand_;

  std::unique_ptr<cosmo::SyncSlave> syncSlave_;
  bool stopUpdating_{false};

  bool isRealRobot_;
  double timeStep_;
  bool publishRos_{false};

  unsigned int actuatorCommandsMissCount_ = 0u;
  cosmo_ros::PublisherRosPtr<SmbCommandsShm, SmbCommandsRos, smb_ros::ConversionTraits> actuatorCommandsPublisher_;

  unsigned int actuatorReadingsMissCount_ = 0u;
  cosmo_ros::SubscriberRosPtr<SmbReadingsShm, SmbReadingsRos, smb_ros::ConversionTraits> actuatorReadingsSubscriber_;

  cosmo_ros::SubscriberRosPtr<SmbStateShm, SmbStateRos, smb_description_ros::ConversionTraits> stateSubscriber_;

  const std::chrono::microseconds receiveMaxLockTime_;
  const std::chrono::microseconds sendMaxLockTime_;
  std::chrono::time_point<std::chrono::steady_clock> t_now_;

  //! Soft emergency stop subscriber which can lead to an emergency stop.
  ros::Subscriber softEmergencyStopSubscriber_;

  //! Hard emergency stop subscriber which can lead to an emergency stop.
  ros::Subscriber hardEmergencyStopSubscriber_;
};

}  // namespace smb_highlevel_controller