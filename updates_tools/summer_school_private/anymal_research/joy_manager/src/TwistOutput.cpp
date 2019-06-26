/*!
* @file     TwistOutput.cpp
* @author   Linus Isler
* @date     Jan, 2017
* @brief
*/

#include <kindr_ros/kindr_ros.hpp>

#include "joy_manager/TwistOutput.hpp"

namespace joy_manager {


TwistOutput::TwistOutput()
: twistMin_(),
  twistMax_()
{}


void TwistOutput::init(const ros::NodeHandle& nh,
                       const std::string& name,
                       const std::string& topic,
                       bool publish) {
  Output::init(nh, name, topic, publish);

  loadParams(nh_);

  auto twistOptions = std::make_shared<cosmo_ros::PublisherRosOptions>(topic, nh_);
  twistOptions->rosQueueSize_ = 10u;
  twistOptions->rosLatch_ = false;
  twistOptions->autoPublishRos_ = false;

  twistPublisher_ = cosmo_ros::advertiseShmRos<TwistShm, TwistRos, any_measurements_ros::ConversionTraits>(
      twistOptions);

  twistMinSubscriber_ = nh_.subscribe(twistMinTopic_, 10, &TwistOutput::twistMinCallback, this);
  twistMaxSubscriber_ = nh_.subscribe(twistMaxTopic_, 10, &TwistOutput::twistMaxCallback, this);
}


void TwistOutput::cleanup() {

}


void TwistOutput::publish(const sensor_msgs::Joy& msg) {
  if (isPublishing_) {
    mapJoyToTwist(msg);
  }
}


void TwistOutput::twistMinCallback(const TwistRos::ConstPtr& msg) {
  boost::unique_lock<boost::shared_mutex> lock(twistMinMutex_);
  kindr::Velocity3D linear;
  kindr::LocalAngularVelocityD angular;
  kindr_ros::convertFromRosGeometryMsg(msg->twist.linear, linear);
  kindr_ros::convertFromRosGeometryMsg(msg->twist.angular, angular);
  twistMin_ = kindr::TwistLocalD(linear, angular);
}


void TwistOutput::twistMaxCallback(const TwistRos::ConstPtr& msg) {
  boost::unique_lock<boost::shared_mutex> lock(twistMaxMutex_);
  kindr::Velocity3D linear;
  kindr::LocalAngularVelocityD angular;
  kindr_ros::convertFromRosGeometryMsg(msg->twist.linear, linear);
  kindr_ros::convertFromRosGeometryMsg(msg->twist.angular, angular);
  twistMax_ = kindr::TwistLocalD(linear, angular);
}


void TwistOutput::mapJoyToTwist(const sensor_msgs::Joy& msg) {
  if ((twistPublisher_->getNumSubscribers() > 0u) && !msg.axes.empty()) {
    boost::shared_lock<boost::shared_mutex> lockMin(twistMinMutex_);
    boost::shared_lock<boost::shared_mutex> lockMax(twistMaxMutex_);

    kindr::Velocity3D linear;
    kindr::LocalAngularVelocityD angular;

    if (msg.axes[HEADING] >= 0.0)
      linear.x() = msg.axes[HEADING] * twistMax_.getTranslationalVelocity().x();
    else
      linear.x() = -msg.axes[HEADING] * twistMin_.getTranslationalVelocity().x();
    if (msg.axes[LATERAL] >= 0.0)
      linear.y() = msg.axes[LATERAL] * twistMax_.getTranslationalVelocity().y();
    else
      linear.y() = -msg.axes[LATERAL] * twistMin_.getTranslationalVelocity().y();
    if (msg.axes[VERTICAL] >= 0.0)
      linear.z() = msg.axes[VERTICAL] * twistMax_.getTranslationalVelocity().z();
    else
      linear.z() = -msg.axes[VERTICAL] * twistMin_.getTranslationalVelocity().z();
    if (msg.axes[ROLL] >= 0.0)
      angular.x() = msg.axes[ROLL] * twistMax_.getRotationalVelocity().x();
    else
      angular.x() = -msg.axes[ROLL] * twistMin_.getRotationalVelocity().x();
    if (msg.axes[PITCH] >= 0.0)
      angular.y() = msg.axes[PITCH] * twistMax_.getRotationalVelocity().y();
    else
      angular.y() = -msg.axes[PITCH] * twistMin_.getRotationalVelocity().y();
    if (msg.axes[TURNING] >= 0.0)
      angular.z() = msg.axes[TURNING] * twistMax_.getRotationalVelocity().z();
    else
      angular.z() = -msg.axes[TURNING] * twistMin_.getRotationalVelocity().z();

    TwistRos twistMsg;
    kindr_ros::convertToRosGeometryMsg(linear, twistMsg.twist.linear);
    kindr_ros::convertToRosGeometryMsg(angular, twistMsg.twist.angular);
    twistMsg.header.stamp = ros::Time::now();
    twistMsg.header.frame_id = frameIdName_;
    TwistShm twistShm = any_measurements_ros::fromRos<TwistShm, TwistRos>(twistMsg);
    twistPublisher_->publishAndSend(twistShm, twistMsg);
  }
}

void TwistOutput::loadParams(const ros::NodeHandle& nh) {
  boost::unique_lock<boost::shared_mutex> lockTwistMin(twistMinMutex_);
  boost::unique_lock<boost::shared_mutex> lockTwistMax(twistMaxMutex_);
  kindr::Velocity3D linear;
  kindr::LocalAngularVelocityD angular;
  nh.param<double>(name_ + "/twistMin/linear/x", linear.x(), -0.0);
  nh.param<double>(name_ + "/twistMin/linear/y", linear.y(), -0.0);
  nh.param<double>(name_ + "/twistMin/linear/z", linear.z(), -0.0);
  nh.param<double>(name_ + "/twistMin/angular/x", angular.x(), -0.0);
  nh.param<double>(name_ + "/twistMin/angular/y", angular.y(), -0.0);
  nh.param<double>(name_ + "/twistMin/angular/z", angular.z(), -0.0);
  twistMin_ = kindr::TwistLocalD(linear, angular);
  nh.param<double>(name_ + "/twistMax/linear/x", linear.x(), 0.0);
  nh.param<double>(name_ + "/twistMax/linear/y", linear.y(), 0.0);
  nh.param<double>(name_ + "/twistMax/linear/z", linear.z(), 0.0);
  nh.param<double>(name_ + "/twistMax/angular/x", angular.x(), 0.0);
  nh.param<double>(name_ + "/twistMax/angular/y", angular.y(), 0.0);
  nh.param<double>(name_ + "/twistMax/angular/z", angular.z(), 0.0);
  twistMax_ = kindr::TwistLocalD(linear, angular);
  nh.param<std::string>(name_ + "/subscribers/twist_min/topic", twistMinTopic_, "/twist_min");
  nh.param<std::string>(name_ + "/subscribers/twist_max/topic", twistMaxTopic_, "/twist_max");
  nh.param<std::string>(name_ + "/frame_id", frameIdName_, "base");
}

} // namespace joy_manager
