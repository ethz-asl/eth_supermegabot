/*!
* @file     SmbTfPublisher.cpp
* @author   Johannes Pankert
* @date     June 9, 2019
* @brief
*/
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "smb_tf_publisher/SmbTfPublisher.hpp"

namespace smb_tf_publisher {

SmbTfPublisher::SmbTfPublisher(any_node::Node::NodeHandlePtr nh) : any_node::Node(nh)
{

}

bool SmbTfPublisher::init() {
  const double publishFrequency = param<double>("publish_frequency", 100);

  baseFrameId_ = param<std::string>("base_frame_id", "base");
  baseFrameIdPrefix_ = param<std::string>("base_frame_id_prefix", "");

  auto qsOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<SmbStateShm>>("smb_state", std::bind(&SmbTfPublisher::smbStateCallback, this, std::placeholders::_1), getNodeHandle());
  qsOptions->autoSubscribe_ = false;
  qsOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  qsOptions->tryRosResubscribing_ = true;

  stateSubscriber_ = cosmo_ros::subscribeShmRos<SmbStateShm,
          SmbStateRos,
          smb_description_ros::ConversionTraits>("smb_state",qsOptions);;

  ignoreState_ = param<bool>("ignore_state", false);
  publishMapTransforms_ = param<bool>("publish_map_transforms", false);

//  anymal_description_ros::initializeFrameTransforms(rosTransforms_);
  rosTransforms_.clear();

  geometry_msgs::TransformStamped transform;
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;

  transform.header.frame_id = "world";
  transform.child_frame_id = "map";
  rosTransforms_.push_back(transform);

  transform.header.frame_id = "world";
  transform.child_frame_id = "map_ga";
  rosTransforms_.push_back(transform);

  // Initialize joint positions message
/*  for (const auto& jointKey : WD::getJointKeys()) {
    jointPositions_.insert(std::pair<std::string, double>(jointKey.getName(), 1.0));
    MELO_INFO_STREAM("Joint name: ");
  }*/
  //joint keys were not set up properly in the description. Use these hardcoded values instead
  jointPositions_.insert(std::pair<std::string, double>("LF_WHEEL", 1.0));
  jointPositions_.insert(std::pair<std::string, double>("LH_WHEEL", 1.0));
  jointPositions_.insert(std::pair<std::string, double>("RF_WHEEL", 1.0));
  jointPositions_.insert(std::pair<std::string, double>("RH_WHEEL", 1.0));


  MELO_INFO_STREAM("Number of joints: " << jointPositions_.size());

  // Initialize robot state publisher
  tf_prefix_ = param<std::string>("tf_prefix", "");
  std::string urdfName = param<std::string>("robot_description", "");
  urdf::Model model;
  if (!model.initParam(urdfName)) {
    MELO_WARN("URDF model load was NOT successful");
    return false;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return false;
  }

  robotStatePublisher_ = std::shared_ptr<robot_state_publisher::RobotStatePublisher>(new robot_state_publisher::RobotStatePublisher(tree));
  robotStatePublisher_->publishFixedTransforms(tf_prefix_, true);
  // Update worker
  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = ros::this_node::getName() + std::string{"_broadcast"};
  workerOptions.callback_ = boost::bind(&SmbTfPublisher::update, this, _1);
  workerOptions.timeStep_ = 1.0/publishFrequency;
  workerOptions.defaultPriority_ = 0;

  if (!addWorker(workerOptions)) {
    ROS_ERROR_STREAM("Could not add worker: " << workerOptions.name_);
    return false;
  }
  return true;
}

void SmbTfPublisher::cleanup() {
  stateSubscriber_->stop();
}

void SmbTfPublisher::smbStateCallback(const SmbStateShm& state) {
  {
    std::lock_guard<std::mutex> lockModel(mutexSmbState_);
    smbStateShm_ = state;
    newState_ = true;
  }
}

bool SmbTfPublisher::update(const any_worker::WorkerEvent& event) {
  // Check if a smb state has been received
  stateSubscriber_->receive();
  if (newState_)
  {
    std::lock_guard<std::mutex> lockModel(mutexSmbState_);
    ros::Time timeMsg = any_measurements_ros::toRos(smbStateShm_.time_);

    if (!smbStateShm_.time_.isZero() &&
        (ignoreState_ || smbStateShm_.status_ >= smb_description::StateStatus::STATUS_OK)) {
      // Broadcast transformation from smb state frame to robot base.
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = timeMsg;
      transform.header.frame_id = "world";
      transform.child_frame_id = tf::resolve(baseFrameIdPrefix_, baseFrameId_);
      kindr_ros::convertToRosGeometryMsg(smbStateShm_.smbState_.getPositionWorldToBaseInWorldFrame(), transform.transform.translation);
      kindr_ros::convertToRosGeometryMsg(smbStateShm_.smbState_.getOrientationBaseToWorld(), transform.transform.rotation);
      tfBroadcaster_.sendTransform(transform);

      // if (publishMapTransforms_) {
      //   try {
      //     rosTransforms_[0].header.stamp = timeMsg;
      //     kindr_ros::convertToRosGeometryMsg(smbStateShm_.smbState_.getFrameTransform(WT::FrameTransformEnum::MapToOdom), rosTransforms_[0].transform);
      //     tfBroadcaster_.sendTransform(rosTransforms_[0]);
      //   } catch (...) {
      //     ROS_ERROR("SmbTfPublisher: Could not get frame map");
      //   }
      //   try {
      //     rosTransforms_[1].header.stamp = timeMsg;
      //     kindr_ros::convertToRosGeometryMsg(smbStateShm_.smbState_.getFrameTransform(WT::FrameTransformEnum::MapGaToOdom), rosTransforms_[1].transform);
      //     tfBroadcaster_.sendTransform(rosTransforms_[1]);
      //   } catch (...) {
      //     ROS_ERROR("SmbTfPublisher: Could not get frame map_ga");
      //   }
      // }
    }
    robotStatePublisher_->publishTransforms(jointPositions_, timeMsg, tf_prefix_);
    newState_ = false;
  }

  return true;
}

} /* namespace smb_tf_publisher */
