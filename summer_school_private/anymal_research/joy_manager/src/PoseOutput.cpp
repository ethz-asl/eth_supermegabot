/*!
* @file     PoseOutput.cpp
* @author   Linus Isler
* @date     Jan, 2017
* @brief
*/

#include <kindr_ros/kindr_ros.hpp>

#include "joy_manager/PoseOutput.hpp"

namespace joy_manager {


PoseOutput::PoseOutput()
: poseMin_(),
  poseMax_()
{}


void PoseOutput::init(const ros::NodeHandle& nh,
                      const std::string& name,
                      const std::string& topic,
                      bool publish) {
  Output::init(nh, name, topic, publish);

  loadParams(nh_);

  auto poseOptions = std::make_shared<cosmo_ros::PublisherRosOptions>(topic, nh_);
  poseOptions->rosQueueSize_ = 10u;
  poseOptions->rosLatch_ = false;
  poseOptions->autoPublishRos_ = false;

  posePublisher_ = cosmo_ros::advertiseShmRos<PoseShm, PoseRos, any_measurements_ros::ConversionTraits>(
      poseOptions);

  poseMinSubscriber_ = nh_.subscribe(poseMinTopic_, 10, &PoseOutput::poseMinCallback, this);
  poseMaxSubscriber_ = nh_.subscribe(poseMaxTopic_, 10, &PoseOutput::poseMaxCallback, this);
}


void PoseOutput::cleanup() {

}


void PoseOutput::publish(const sensor_msgs::Joy& msg) {
  if (isPublishing_) {
    mapJoyToPose(msg);
  }
}


void PoseOutput::poseMinCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  boost::unique_lock<boost::shared_mutex> lock(poseMinMutex_);
  kindr_ros::convertFromRosGeometryMsg(msg->pose, poseMin_);
}


void PoseOutput::poseMaxCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  boost::unique_lock<boost::shared_mutex> lock(poseMaxMutex_);
  kindr_ros::convertFromRosGeometryMsg(msg->pose, poseMax_);
}


void PoseOutput::mapJoyToPose(const sensor_msgs::Joy& msg) {
  if ((posePublisher_->getNumSubscribers() > 0u) && !msg.axes.empty()) {
    boost::shared_lock<boost::shared_mutex> lockMin(poseMinMutex_);
    boost::shared_lock<boost::shared_mutex> lockMax(poseMaxMutex_);

    kindr::Position3D position;
//    if (msg.axes[HEADING] >= 0.0)
//      position.x() = msg.axes[HEADING] * poseMax_.getPosition().x();
//    else
//      position.x() = -msg.axes[HEADING] * poseMin_.getPosition().x();
//    if (msg.axes[LATERAL] >= 0.0)
//      position.y() = msg.axes[LATERAL] * poseMax_.getPosition().y();
//    else
//      position.y() = -msg.axes[LATERAL] * poseMin_.getPosition().y();
    if (msg.axes[VERTICAL] >= 0.0)
      position.z() = msg.axes[VERTICAL] * poseMax_.getPosition().z();
    else
      position.z() = -msg.axes[VERTICAL] * poseMin_.getPosition().z();

    kindr::EulerAnglesZyxD maxAngles(poseMax_.getRotation());
    maxAngles = maxAngles.getUnique();
    kindr::EulerAnglesZyxD minAngles(poseMin_.getRotation());
    minAngles = minAngles.getUnique();
    double roll, pitch, yaw;

    if (msg.axes[LATERAL] >= 0.0)
      roll = msg.axes[LATERAL] * maxAngles.roll();
    else
      roll = -msg.axes[LATERAL] * minAngles.roll();
    if (msg.axes[HEADING] >= 0.0)
      pitch = -msg.axes[HEADING] * maxAngles.pitch();
    else
      pitch = msg.axes[HEADING] * minAngles.pitch();
    if (msg.axes[TURNING] >= 0.0)
      yaw = msg.axes[TURNING] * maxAngles.yaw();
    else
      yaw = -msg.axes[TURNING] * minAngles.yaw();

    kindr::EulerAnglesZyxD orientationEuler(yaw, pitch, roll);
    orientationEuler = orientationEuler.getUnique();
    kindr::RotationQuaternionD orientationQuatD(orientationEuler);
    kindr::HomTransformQuatD pose(position, orientationQuatD);
    PoseRos poseMsg;
    kindr_ros::convertToRosGeometryMsg(pose, poseMsg.pose);
    auto timeNow = ros::Time::now();
    poseMsg.header.stamp = timeNow;
    poseMsg.header.frame_id = frameIdName_;
    PoseShm poseShm(any_measurements_ros::fromRos<any_measurements::Time, ros::Time>(timeNow), pose);
    posePublisher_->publishAndSend(poseShm, poseMsg);
  }
}


void PoseOutput::loadParams(const ros::NodeHandle& nh) {
  boost::unique_lock<boost::shared_mutex> lockPoseMin(poseMinMutex_);
  boost::unique_lock<boost::shared_mutex> lockPoseMax(poseMaxMutex_);
  kindr::Position3D position;
  double x, y, z, w;
  nh.param<double>(name_ + "/poseMin/position/x", position.x(), -0.0);
  nh.param<double>(name_ + "/poseMin/position/y", position.y(), -0.0);
  nh.param<double>(name_ + "/poseMin/position/z", position.z(), -0.0);
  nh.param<double>(name_ + "/poseMin/orientation/x", x, 0.0);
  nh.param<double>(name_ + "/poseMin/orientation/y", y, 0.0);
  nh.param<double>(name_ + "/poseMin/orientation/z", z, 0.0);
  nh.param<double>(name_ + "/poseMin/orientation/w", w, 1.0);
  kindr::QuaternionD quaternion(w, x, y, z);
  kindr::RotationQuaternionD orientation(quaternion.toUnitQuaternion());
  poseMin_ = kindr::HomTransformQuatD(position, orientation);
  nh.param<double>(name_ + "/poseMax/position/x", position.x(), 0.0);
  nh.param<double>(name_ + "/poseMax/position/y", position.y(), 0.0);
  nh.param<double>(name_ + "/poseMax/position/z", position.z(), 0.0);
  nh.param<double>(name_ + "/poseMax/orientation/x", x, 0.0);
  nh.param<double>(name_ + "/poseMax/orientation/y", y, 0.0);
  nh.param<double>(name_ + "/poseMax/orientation/z", z, 0.0);
  nh.param<double>(name_ + "/poseMax/orientation/w", w, 1.0);
  quaternion = kindr::QuaternionD(w, x, y, z);
  orientation = kindr::RotationQuaternionD(quaternion.toUnitQuaternion());
  poseMax_ = kindr::HomTransformQuatD(position, orientation);
  nh.param<std::string>(name_ + "/subscribers/pose_min/topic", poseMinTopic_, "/pose_min");
  nh.param<std::string>(name_ + "/subscribers/pose_max/topic", poseMaxTopic_, "/pose_max");
  nh.param<std::string>(name_ + "/frame_id", frameIdName_, "base");
}

} // namespace joy_manager
