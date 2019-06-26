/*
 * Copyright 2018 Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef INCLUDE_CONFUSION_ROSCONVERSIONS_H_
#define INCLUDE_CONFUSION_ROSCONVERSIONS_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
//#include "tf_conversions/tf_eigen.h"

#include "confusion/utilities/Pose.h"
#include "confusion/models/ImuMeas.h"


namespace confusion {

inline geometry_msgs::Vector3 VectorToRosMsg(const Eigen::Vector3d& vector)
{
	geometry_msgs::Vector3 msg_point;
	msg_point.x = vector.x();
	msg_point.y = vector.y();
	msg_point.z = vector.z();
	return msg_point;
}

inline Eigen::Vector3d RosMsgToVector(const geometry_msgs::Vector3& msg_point)
{
	Eigen::Vector3d vector;
	vector.x() = msg_point.x;
	vector.y() = msg_point.y;
	vector.z() = msg_point.z;
	return vector;
}

inline geometry_msgs::Quaternion QuaternionToRosMsg(const Eigen::Quaterniond& quat)
{
	geometry_msgs::Quaternion msg_quat;
	msg_quat.w = quat.w();
	msg_quat.x = quat.x();
	msg_quat.y = quat.y();
	msg_quat.z = quat.z();
	return msg_quat;
}

inline Eigen::Quaterniond RosMsgToQuaternion(const geometry_msgs::Quaternion& msg_quat)
{
	Eigen::Quaterniond quat;
	quat.w() = msg_quat.w;
	quat.x() = msg_quat.x;
	quat.y() = msg_quat.y;
	quat.z() = msg_quat.z;
	return quat;
}

inline geometry_msgs::Pose getMsg(Pose<double> T) {
	geometry_msgs::Pose msg;
	msg.position.x = T.trans[0];
	msg.position.y = T.trans[1];
	msg.position.z = T.trans[2];
	msg.orientation.w = T.rot.w();
	msg.orientation.x = T.rot.x();
	msg.orientation.y = T.rot.y();
	msg.orientation.z = T.rot.z();
	return msg;
}

inline geometry_msgs::PoseStamped getMsg(Pose<double> T, std::string frame_id,
		ros::Time stamp) {
	geometry_msgs::PoseStamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = frame_id;
	msg.pose.position.x = T.trans[0];
	msg.pose.position.y = T.trans[1];
	msg.pose.position.z = T.trans[2];
	msg.pose.orientation.w = T.rot.w();
	msg.pose.orientation.x = T.rot.x();
	msg.pose.orientation.y = T.rot.y();
	msg.pose.orientation.z = T.rot.z();
	return msg;
}

inline geometry_msgs::PoseStamped getMsg(Pose<double> T, std::string frame_id) {
	return getMsg(T, frame_id, ros::Time::now());
}

inline Pose<double> getPoseFromMsg(geometry_msgs::Pose msg) {
	return Pose<double>(msg.position.x, msg.position.y, msg.position.z,
			msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
}
inline Pose<double> getPoseFromMsg(geometry_msgs::Transform msg) {
	return Pose<double>(msg.translation.x, msg.translation.y, msg.translation.z,
			msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
}

inline Pose<double> getPoseFromMsg(const tf::Transform &msg) {
  return Pose<double>(msg.getOrigin().x(), msg.getOrigin().y(), msg.getOrigin().z(),
                      msg.getRotation().w(), msg.getRotation().x(), msg.getRotation().y(), msg.getRotation().z());
}
inline Pose<double> getPoseFromMsg(const tf::StampedTransform &msg) {
  return Pose<double>(msg.getOrigin().x(), msg.getOrigin().y(), msg.getOrigin().z(),
                      msg.getRotation().w(), msg.getRotation().x(), msg.getRotation().y(), msg.getRotation().z());
}

inline tf::StampedTransform getTfStampedMsg(Pose<double> T_a_b, std::string parentFrameId, std::string childFrameId) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(T_a_b.trans(0), T_a_b.trans(1), T_a_b.trans(2)));
  transform.setRotation(tf::Quaternion(T_a_b.rot.x(), T_a_b.rot.y(), T_a_b.rot.z(), T_a_b.rot.w())); //Careful! Order is x,y,z,w
  return tf::StampedTransform(transform, ros::Time::now(), parentFrameId, childFrameId);
}

inline tf::Transform getTfMsg(Pose<double> T_a_b) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(T_a_b.trans(0), T_a_b.trans(1), T_a_b.trans(2)));
  transform.setRotation(tf::Quaternion(T_a_b.rot.x(), T_a_b.rot.y(), T_a_b.rot.z(), T_a_b.rot.w())); //Careful! Order is x,y,z,w
  return transform;
}

inline Pose<double> getPoseFromMsg(const geometry_msgs::TransformStampedPtr &msg) {
	return Pose<double>(msg->transform.translation.x,
											msg->transform.translation.y,
											msg->transform.translation.z,
											msg->transform.rotation.w,
											msg->transform.rotation.x,
											msg->transform.rotation.y,
											msg->transform.rotation.z);
}

inline Pose<double> getPoseFromMsg(const nav_msgs::OdometryPtr &msg) {
	return Pose<double>(msg->pose.pose.position.x,
											msg->pose.pose.position.y,
											msg->pose.pose.position.z,
											msg->pose.pose.orientation.w,
											msg->pose.pose.orientation.x,
											msg->pose.pose.orientation.y,
											msg->pose.pose.orientation.z);
}

inline nav_msgs::Odometry getOdometryMsg(double t, Pose<double> T_w_i,
		Eigen::Vector3d angVel, Eigen::Vector3d linVel) {
	nav_msgs::Odometry msg;
	msg.header.stamp = ros::Time(t);
	msg.header.frame_id = "/world";
	msg.pose.pose = getMsg(T_w_i);
	msg.twist.twist.angular.x = angVel(0);
	msg.twist.twist.angular.y = angVel(1);
	msg.twist.twist.angular.z = angVel(2);
	msg.twist.twist.linear.x = linVel(0);
	msg.twist.twist.linear.y = linVel(1);
	msg.twist.twist.linear.z = linVel(2);

	return msg;
}

//todo Also output a timestamp somehow?
inline std_msgs::Float64MultiArray matrixToRosMsg(std::string topic, Eigen::MatrixXd mat,
		std::string label = "") {
	int cols = mat.cols();
	int rows = mat.rows();

	std_msgs::Float64MultiArray msg;
	msg.layout.dim.resize(2);
	msg.layout.dim[0].label = label;
	msg.layout.dim[0].size = cols;
	msg.layout.dim[0].stride = cols * rows;
	msg.layout.dim[1].label = "mat";
	msg.layout.dim[1].size = rows;
	msg.layout.dim[1].stride = rows;
	msg.data.resize(cols * rows);

	//todo Can just copy the data directly in one line?
	for(int i=0; i<cols; ++i) {
		for (int j=0; j<rows; ++j) {
		msg.data[i*rows + j] = mat(j,i);
		}
	}

	return msg;
}

inline std_msgs::Float64MultiArray vectorToRosMsg(std::string topic, Eigen::VectorXd vec,
		double stamp, std::string label = "") {
	int size = vec.size()+1;

	std_msgs::Float64MultiArray msg;
	msg.layout.dim.resize(1);
	msg.layout.dim[0].label = label;
	msg.layout.dim[0].size = size;
	msg.layout.dim[0].stride = size;
	msg.data.resize(size);

	msg.data[0] = stamp;
	for(int i=0; i<vec.size(); ++i) {
		msg.data[i+1] = vec(i);
	}

	return msg;
}

/**
 * Convert a ROS IMU message to ConFusion type. Note that this does not fill in the calibration struct or gravity orientation parameter reference.
 * @param msg The ROS message
 * @return The ConFusion-type IMU measurement
 */
inline ImuMeas rosmsgToConfusionImuMeas(const sensor_msgs::Imu& msg) {
  Eigen::Vector3d a(msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z);
  Eigen::Vector3d w(msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z);
  return confusion::ImuMeas(msg.header.stamp.toSec(), a, w);
}

/**
 * Convert a ROS IMU message to ConFusion type. Note that this does not fill in the calibration struct or gravity orientation parameter reference.
 * @param meas The IMU measurement
 * @return The ROS message
 */
inline sensor_msgs::Imu confusionImuMeasToRosmsg(const ImuMeas& meas) {
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time(meas.t());
  msg.linear_acceleration.x = meas.a_(0);
  msg.linear_acceleration.y = meas.a_(1);
  msg.linear_acceleration.z = meas.a_(2);
  msg.angular_velocity.x = meas.w_(0);
  msg.angular_velocity.y = meas.w_(1);
  msg.angular_velocity.z = meas.w_(2);
  return msg;
}

}



#endif /* INCLUDE_CONFUSION_ROSCONVERSIONS_H_ */
