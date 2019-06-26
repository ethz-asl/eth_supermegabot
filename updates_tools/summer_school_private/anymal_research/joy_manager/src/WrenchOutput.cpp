/*!
* @file     WrenchOutput.cpp
* @author   Koen Krämer
* @date     Nov, 2017
* @brief
*/

#include <kindr_ros/kindr_ros.hpp>

#include "joy_manager/WrenchOutput.hpp"

namespace joy_manager {

    WrenchOutput::WrenchOutput()
            : wrenchMin_(),
              wrenchMax_()
    {}

    void WrenchOutput::init(const ros::NodeHandle& nh,
                            const std::string& name,
                            const std::string& topic,
                            bool publish) {
      Output::init(nh, name, topic, publish);

      loadParams(nh_);

      auto wrenchOptions = std::make_shared<cosmo_ros::PublisherRosOptions>(topic, nh_);
      wrenchOptions->rosQueueSize_ = 10u;
      wrenchOptions->rosLatch_ = false;
      wrenchOptions->autoPublishRos_ = false;

      wrenchPublisher_ = cosmo_ros::advertiseShmRos<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits>(
              wrenchOptions);
      wrenchMinSubscriber_ = nh_.subscribe(wrenchMinTopic_, 10, &WrenchOutput::wrenchMinCallback, this);
      wrenchMaxSubscriber_ = nh_.subscribe(wrenchMaxTopic_, 10, &WrenchOutput::wrenchMaxCallback, this);
    }

    void WrenchOutput::cleanup() {

    }

    void WrenchOutput::publish(const sensor_msgs::Joy& msg) {
      if (isPublishing_) {
        mapJoyToWrench(msg);
      }
    }

    void WrenchOutput::wrenchMinCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
      boost::unique_lock<boost::shared_mutex> lock(wrenchMinMutex_);
      kindr::Force3D force;
      kindr::Torque3D torque;
      kindr_ros::convertFromRosGeometryMsg(msg->wrench.force, force);
      kindr_ros::convertFromRosGeometryMsg(msg->wrench.torque, torque);
      wrenchMin_ = kindr::WrenchD(force, torque);
    }

    void WrenchOutput::wrenchMaxCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
      boost::unique_lock<boost::shared_mutex> lock(wrenchMaxMutex_);
      kindr::Force3D force;
      kindr::Torque3D torque;
      kindr_ros::convertFromRosGeometryMsg(msg->wrench.force, force);
      kindr_ros::convertFromRosGeometryMsg(msg->wrench.torque, torque);
      wrenchMax_ = kindr::WrenchD(force, torque);
    }

    void WrenchOutput::mapJoyToWrench(const sensor_msgs::Joy& msg) {
      if (wrenchPublisher_->getNumSubscribers() > 0u && !msg.axes.empty()) {
        boost::shared_lock<boost::shared_mutex> lockMin(wrenchMinMutex_);
        boost::shared_lock<boost::shared_mutex> lockMax(wrenchMaxMutex_);

        kindr::Force3D force;
        kindr::Torque3D torque;

        if (msg.axes[HEADING] >= 0.0)
          force.x() = msg.axes[HEADING] * wrenchMax_.getForce().x();
        else
          force.x() = -msg.axes[HEADING] * wrenchMin_.getForce().x();
        if (msg.axes[LATERAL] >= 0.0)
          force.y() = msg.axes[LATERAL] * wrenchMax_.getForce().y();
        else
          force.y() = -msg.axes[LATERAL] * wrenchMin_.getForce().y();
        if (msg.axes[VERTICAL] >= 0.0)
          force.z() = msg.axes[VERTICAL] * wrenchMax_.getForce().z();
        else
          force.z() = -msg.axes[VERTICAL] * wrenchMin_.getForce().z();
        if (msg.axes[ROLL] >= 0.0)
          torque.x() = msg.axes[ROLL] * wrenchMax_.getTorque().x();
        else
          torque.x() = -msg.axes[ROLL] * wrenchMin_.getTorque().x();
        if (msg.axes[PITCH] >= 0.0)
          torque.y() = msg.axes[PITCH] * wrenchMax_.getTorque().y();
        else
          torque.y() = -msg.axes[PITCH] * wrenchMin_.getTorque().y();
        if (msg.axes[TURNING] >= 0.0)
          torque.z() = msg.axes[TURNING] * wrenchMax_.getTorque().z();
        else
          torque.z() = -msg.axes[TURNING] * wrenchMin_.getTorque().z();

        WrenchRos wrenchMsg;
        kindr_ros::convertToRosGeometryMsg(force, wrenchMsg.wrench.force);
        kindr_ros::convertToRosGeometryMsg(torque, wrenchMsg.wrench.torque);
        wrenchMsg.header.stamp = ros::Time::now();
        wrenchMsg.header.frame_id = frameIdName_;
        WrenchShm wrenchShm = any_measurements_ros::fromRos<WrenchShm, WrenchRos>(wrenchMsg);
        wrenchPublisher_->publishAndSend(wrenchShm, wrenchMsg);
      }
    }

    void WrenchOutput::loadParams(const ros::NodeHandle& nh) {
      boost::unique_lock<boost::shared_mutex> lockWrenchMin(wrenchMinMutex_);
      boost::unique_lock<boost::shared_mutex> lockWrenchMax(wrenchMaxMutex_);
      kindr::Force3D force;
      kindr::Torque3D torque;
      nh.param<double>(name_ + "/wrenchMin/force/x", force.x(), -0.0);
      nh.param<double>(name_ + "/wrenchMin/force/y", force.y(), -0.0);
      nh.param<double>(name_ + "/wrenchMin/force/z", force.z(), -0.0);
      nh.param<double>(name_ + "/wrenchMin/torque/x", torque.x(), -0.0);
      nh.param<double>(name_ + "/wrenchMin/torque/y", torque.y(), -0.0);
      nh.param<double>(name_ + "/wrenchMin/torque/z", torque.z(), -0.0);
      wrenchMin_ = kindr::WrenchD(force, torque);
      nh.param<double>(name_ + "/wrenchMax/force/x", force.x(), 0.0);
      nh.param<double>(name_ + "/wrenchMax/force/y", force.y(), 0.0);
      nh.param<double>(name_ + "/wrenchMax/force/z", force.z(), 0.0);
      nh.param<double>(name_ + "/wrenchMax/torque/x", torque.x(), 0.0);
      nh.param<double>(name_ + "/wrenchMax/torque/y", torque.y(), 0.0);
      nh.param<double>(name_ + "/wrenchMax/torque/z", torque.z(), 0.0);
      wrenchMax_ = kindr::WrenchD(force, torque);
      nh.param<std::string>(name_ + "/subscribers/wrench_min/topic", wrenchMinTopic_, "/wrench_min");
      nh.param<std::string>(name_ + "/subscribers/wrench_max/topic", wrenchMaxTopic_, "/wrench_max");
      nh.param<std::string>(name_ + "/frame_id", frameIdName_, "base");
    }

} // namespace joy_manager



