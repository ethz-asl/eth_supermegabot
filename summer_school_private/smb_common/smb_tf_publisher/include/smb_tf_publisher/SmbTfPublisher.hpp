/*!
* @file     SmbTfPublisher.hpp
* @author   Johannes Pankert
* @date     June 9, 2019
* @brief
*/
#pragma once

#include <ros/ros.h>
#include <any_node/any_node.hpp>

#include <smb_msgs/SmbState.h>
#include <tf/transform_broadcaster.h>
#include <mutex>

#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
/*#include <smb_description/Containers.hpp>
#include <smb_description_ros/ConversionTraits.hpp>*/
#include <smb_description_ros/smb_description_ros.hpp>
#include <any_measurements_ros/ConvertRosMessages.hpp>
#include <message_logger/message_logger.hpp>

//#include <anymal_description_ros/anymal_description_ros.hpp>
#include <kindr_ros/kindr_ros.hpp>

#include <cosmo_ros/cosmo_ros.hpp>

namespace smb_tf_publisher {

//! This currently only re-publishes the robot state to two separate topics: pose and joint state.
class SmbTfPublisher : public any_node::Node
{
 public:
  using WD = smb_model::SmbModel::SmbDescription;
  using WT = smb_description::SmbTopology;
  typedef smb_description::SmbState SmbStateShm;
  typedef smb_msgs::SmbState SmbStateRos;

  explicit SmbTfPublisher(any_node::Node::NodeHandlePtr nh);
  ~SmbTfPublisher() override = default;

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent& event);

  void smbStateCallback(const SmbStateShm& state);

 protected:
  std::string baseFrameId_;
  std::string baseFrameIdPrefix_;
  cosmo_ros::SubscriberRosPtr<SmbStateShm, SmbStateRos,
    smb_description_ros::ConversionTraits> stateSubscriber_;
  SmbStateShm smbStateShm_;
  SmbStateRos smbStateRos_;
  std::atomic<bool> newState_;
  std::vector<geometry_msgs::TransformStamped> rosTransforms_;
  std::mutex mutexSmbState_;
  tf::TransformBroadcaster tfBroadcaster_;
  bool ignoreState_ = false;
  bool publishMapTransforms_ = false;
  ros::Publisher posePublisher_;
  ros::Publisher jointPublisher_;

  std::map<std::string, double> jointPositions_;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisher_;
  std::string tf_prefix_;
};

} /* namespace smb_tf_publisher */
