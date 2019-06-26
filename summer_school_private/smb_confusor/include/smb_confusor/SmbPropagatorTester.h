//
// Created by tim on 28.01.19.
//

#ifndef SMB_SMBPROPAGATORTESTER_H
#define SMB_SMBPROPAGATORTESTER_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <confusion/utilities/ros_conversions.h>

#include "smb_confusor/SmbPropagator.h"

namespace smb_confusor {

class SmbPropagatorTester {
 public:
  SmbPropagatorTester(ros::NodeHandle &node);

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void baseOdometryCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void stateEstimateCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

  smb_confusor::SmbPropagator smbPropagator_;

 private:
  confusion::ImuCalibration imuCalibration_;
  Eigen::Vector2d gravity_rot_;
  BaseMotionModelCalibration baseMotionModelCalibration_;
  confusion::Pose<double> T_imu_base_;


  ros::Subscriber subImu_;
  ros::Subscriber baseOdometrySub_;
  ros::Subscriber stateEstimateSub_;
};

} // namespace smb_confusor

#endif //SMB_SMBPROPAGATORTESTER_H
