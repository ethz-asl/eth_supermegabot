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

#ifndef INCLUDE_CONFUSION_ERTMODULE_H_
#define INCLUDE_CONFUSION_ERTMODULE_H_

#include <ros/package.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <boost/circular_buffer.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <mutex>
#include <thread>

#include <confusion/OdomWithState.h>
#include <confusion/TagArray.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <confusion/BatchFusor.h>
#include <confusion/ConFusor.h>
#include <confusion/Logger.h>
#include <confusion/utilities/utilities.h>

#include "confusion/models/PoseMeas.h"
#include "confusion/utilities/ros_conversions.h"

namespace confusion {

//todo Move this somewhere else
inline confusion::Pose<double> getPoseFromMsg(const confusion::OdomWithStatePtr &msg) {
  return confusion::Pose<double>(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y,
                                 msg->pose.pose.position.z,
                                 msg->pose.pose.orientation.w,
                                 msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z);
}



class ExternalReferenceTrackingModule {
 public:
  ExternalReferenceTrackingModule(ros::NodeHandle &node,
                                  ConFusor &conFusor,
                                  std::string configFileName,
                                  int poseMeasIndex,
                                  bool* newMeasReceivedFlag);

  void initializeSensorframeOffset(std::string sensorname);

  template <typename MsgType>
  void externalPoseMeasCallback(const MsgType &msg);

  void externalPoseMeasTFStampedCallback(const geometry_msgs::TransformStampedPtr &msg);

  void externalPoseMeasOdomCallback(const nav_msgs::OdometryPtr &msg);

  void externalPoseMeasOdomWithStateCallback(const confusion::OdomWithStatePtr &msg);

  void copyOutParametersAfterOptimization();
  void publishFrameOffsetTfs();

  //Used to configure the sensor frame offsets for extrinsic calibration
  //Note that this should not be called while sensor fusion is running, since it
  //modifies that state of the optimized parameters (possibly) asynchronously
  void setExtrinsicsUnconstForBatchSolve();
  void resetExtrinsicsConstAfterBatchSolve();

  ros::NodeHandle &node_;
  std::vector<ros::Subscriber> subsExternalPoseMeas_;

  tf::TransformBroadcaster tfBroadcaster_;

  confusion::ConFusor &conFusor_;

  bool getReferenceFrameOffset(const std::string &frameName, confusion::Pose<double> &T_world_ref) const;
  bool getSensorFrameOffset(const std::string &frameName, confusion::Pose<double> &T_body_sensor) const;
  bool getSensorScale(const std::string &frameName, double &scale) const;

  std::map<std::string, std::shared_ptr<confusion::Pose<double>>>* getReferenceFrameOffsetsMapPointer() { return &referenceFrameOffsets_; }

private:
  // Vector of offsets from external reference frames to the estimator world frame (T_world_ref)
  std::map<std::string, std::shared_ptr<confusion::Pose<double>>> referenceFrameOffsets_;
  // Vector of offsets from the body to other sensor frames (T_body_sensor)
  std::map<std::string, std::shared_ptr<confusion::Pose<double>>> sensorFrameOffsets_;
  std::map<std::string, std::shared_ptr<double>> sensorPoseMeasScales_;
  std::map<std::string, std::shared_ptr<confusion::PoseMeasConfig>> sensorPoseMeasConfigs_;

  // Copy of the optimized parameters for asynchronous queries during optimization
  mutable std::mutex parameterQueryMtx_;
  std::map<std::string, confusion::Pose<double>> referenceFrameOffsetsCopy_;
  std::map<std::string, confusion::Pose<double>> sensorFrameOffsetsCopy_;
  std::map<std::string, double> sensorPoseMeasScalesCopy_;

  std::string confusionPath_;
  boost::property_tree::ptree pt;

  // This can be used to indicate when new tag measurements have been added to the ConFusor to tell ongoing optimizations to stop early
  bool *newMeasReceivedFlag_ = nullptr;
  int poseMeasIndex_;
};

}  // namespace confusion

#endif /* INCLUDE_CONFUSION_ERTMODULE_H_ */
