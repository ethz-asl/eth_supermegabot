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

#ifndef INCLUDE_CONFUSION_EXAMPLES_TAGTRACKER_H_
#define INCLUDE_CONFUSION_EXAMPLES_TAGTRACKER_H_

//#define SYNC_DEBUG
//#define COST_DEBUG

#include <thread>
#include <mutex>
#include <Eigen/Core>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include <confusion/ConFusor.h>
#include <confusion/BatchFusor.h>
#include <confusion/Logger.h>
#include <confusion/utilities/utilities.h>

#include "confusion/models/PoseMeas.h"
#include "confusion/TagTrackerParameters.h"
#include "confusion/modules/apriltag/AprilTagModule.h"
#include "confusion/utilities/ros_conversions.h"
#include "confusion/utilities/ImuPropagator.h"
#include "confusion/Diagram.h"

namespace confusion {

template <typename StateType>
class TagTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  TagTracker(ros::NodeHandle &node);

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

  void triggerBatchCalCallback(const std_msgs::Empty &msg);
  void drawDiagramCallback(const std_msgs::Empty &msg);

  virtual void runEstimator();

  void stopTracking(const std_msgs::Empty &msg);

  void startTracking(const std_msgs::Empty &msg);

  void stopTracking();

  void startTracking();

  void publish(std::shared_ptr<confusion::State> statePtr, const confusion::StateVector *stateVector);
  virtual void publishDerived(std::shared_ptr<confusion::State> statePtr, const confusion::StateVector *stateVector) { }

  ros::NodeHandle &node_;
  ros::Subscriber subImu_;
  ros::Subscriber subTriggerBatch_;
  ros::Subscriber subDrawDiagram_;

  image_transport::Publisher imagePub_;
  ros::Publisher pubState_;
  ros::Publisher pubRtState_;
  ros::Publisher pubStates_;
  ros::Publisher pubPose_;
  ros::Publisher pubRtPose_;
  tf::TransformBroadcaster tfBroadcaster_;

  std::thread estimatorThread_;

  ConFusor conFusor_;

  std::unique_ptr<AprilTagModule> aprilTagInterface_;
  TagTrackerParameters tagTrackerParameters_;

  Eigen::Vector2d gravity_rot_;

  int batchSize_;
  bool newTagMeasReady_ = false;

  bool logData_;

 protected:
  bool forwardPropagateState_;
  bool tracking_ = false;
  double t_imu_latest_ = 0.0;

 private:
  std::string confusionPath_;
  std::unique_ptr<confusion::Logger> logger_;
  boost::property_tree::ptree pt;

  //For saving states and running a batch problem at the end
  confusion::StateVector statesBatch_;
  bool runBatch_ = false;
  bool run_ = true;
  bool runEstimatorLoopDone_ = false;
  int stateToDropIndex_ = -1; //-1 means that we shouldn't drop a state
  bool drawDiagramRequest_ = false;

  confusion::ImuPropagator imuPropagator_;

  std::string configFile = "/example/tagtracker_config.cfg";
  double loop_freq = 100; // [Hz]
};

} //namespace confusion

#include "confusion/impl/TagTracker.h"

#endif /* INCLUDE_CONFUSION_EXAMPLES_TAGTRACKER_H_ */
