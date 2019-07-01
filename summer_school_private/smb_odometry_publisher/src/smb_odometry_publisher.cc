/*
 * odometry_publisher.cc
 *
 *  Created on: 26. June, 2019
 *      Author: Florian Tschopp
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>

class OdometryPublisher {
public:
  OdometryPublisher(ros::NodeHandle &node_handle) : node_handle_(node_handle) {
    ROS_INFO("Odometry publisher node started.");
    readParameters();
    wheel_speed_sub_ = node_handle_.subscribe(
        wheel_speed_topic_, 1, &OdometryPublisher::wheelSpeedsCallback, this);
    odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>(odometry_topic_, 1);
  }

  virtual ~OdometryPublisher() { node_handle_.shutdown(); }

  void
  wheelSpeedsCallback(const std_msgs::Float64MultiArray &wheel_speeds_msg) {
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped odom_trans;
    odom.child_frame_id = "/smb/base_link";
    odom.header.frame_id = "/odom";
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "smb/base_link";
    if (last_odometry_.header.stamp.toSec() == 0) {
      // Initialization.
      last_odometry_.header.stamp = ros::Time(wheel_speeds_msg.data[0]);
      last_theta_ = 0.0;
    } else {
      odom.header.stamp = ros::Time(wheel_speeds_msg.data[0]);

      double delta_t_s = (odom.header.stamp.sec * 1E9 + odom.header.stamp.nsec -
                          (last_odometry_.header.stamp.sec * 1E9 +
                           last_odometry_.header.stamp.nsec)) /
                         1E9;

      // Create odometry message.
      odom.twist.twist.linear.x =
          (wheel_speeds_msg.data[2] + wheel_speeds_msg.data[1]) *
          wheel_radius_ / 2.0;
      odom.twist.twist.angular.z =
          (wheel_speeds_msg.data[1] - wheel_speeds_msg.data[2]) *
          wheel_radius_ / baseline_;
      double theta = last_theta_ + odom.twist.twist.angular.z * delta_t_s;
      odom.pose.pose.position.x =
          last_odometry_.pose.pose.position.x +
          odom.twist.twist.linear.x *
              cos(last_theta_ + odom.twist.twist.angular.z * delta_t_s / 2.0) *
              delta_t_s;
      odom.pose.pose.position.y =
          last_odometry_.pose.pose.position.y +
          odom.twist.twist.linear.x *
              sin(last_theta_ + odom.twist.twist.angular.z * delta_t_s / 2.0) *
              delta_t_s;
      odom.pose.pose.position.z = 0.0;

      // since all odometry is 6DOF we'll need a quaternion created from yaw
      ROS_WARN_STREAM("Theta: " << theta);
      geometry_msgs::Quaternion odom_quat =
          tf::createQuaternionMsgFromYaw(theta);
      odom.pose.pose.orientation = odom_quat;

      // Create TF message.
      odom_trans.header = odom.header;
      // Sadly, odom_trans.transform.translation is a geometry_msgs/Vector3
      // while odom.pose.pose.position is a geometry_msgs/Point.
      odom_trans.transform.translation.x = odom.pose.pose.position.x;
      odom_trans.transform.translation.y = odom.pose.pose.position.y;
      odom_trans.transform.translation.z = odom.pose.pose.position.z;
      odom_trans.transform.rotation = odom.pose.pose.orientation;

      // send the transform
      last_theta_ = theta;
      last_odometry_ = odom;
    }
    odom_pub_.publish(odom);
    odom_broadcaster_.sendTransform(odom_trans);
  }

  bool readParameters() {
    ROS_INFO("Read ROS parameters.");
    node_handle_.param("nav_odometry_topic", odometry_topic_,
                       std::string("/odometry"));
    node_handle_.param("wheel_speed_topic", wheel_speed_topic_,
                       std::string("/wheelSpeeds"));
    node_handle_.param("wheel_radius", wheel_radius_, 0.2);
    node_handle_.param("baseline", baseline_, 0.7);
    return true;
  }

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber wheel_speed_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;
  std::string odometry_topic_;
  std::string wheel_speed_topic_;
  double baseline_;
  double wheel_radius_;
  nav_msgs::Odometry last_odometry_;
  double last_theta_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_publisher_node");
  ros::NodeHandle nodeHandle("~");
  OdometryPublisher odometry_publisher(nodeHandle);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
