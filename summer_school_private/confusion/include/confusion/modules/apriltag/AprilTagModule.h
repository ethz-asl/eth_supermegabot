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

#ifndef INCLUDE_CONFUSION_EXAMPLES_APRILTAGINTERFACE_H_
#define INCLUDE_CONFUSION_EXAMPLES_APRILTAGINTERFACE_H_

#include <mutex>
#include <thread>
#include <Eigen/Core>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>
#include "confusion/modules/apriltag/AprilTagParameters.h"
#include "confusion/modules/apriltag/external/april_tag_detector.hpp"
#include <confusion/TagArray.h>
#include <tf/transform_broadcaster.h>

#include <confusion/utilities/utilities.h>

#include "confusion/ConFusor.h"
#include "confusion/models/TagMeas.h"
#include "confusion/utilities/ros_conversions.h"

namespace confusion {

class AprilTagModule {
 public:
  typedef std::map<std::string, std::shared_ptr<confusion::Pose<double>>> FrameOffsetMap;

  AprilTagModule(ros::NodeHandle &node,
                    ConFusor* conFusorPtr,
                    std::string configFileName,
                    int tagMeasIndex_,
                    bool* newMeasReceivedFlag = nullptr);

  ~AprilTagModule();

  void camCalCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

  void processTagDetections(double t, std::vector<TagDetection> &tagDetections);

  void tagArrayCallback(const confusion::TagArray::ConstPtr &msg);

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void tagDetection(const sensor_msgs::ImageConstPtr &msg);

  void publishTagMarkers();

  void setVerbose(bool verbose) { verbose_ = verbose; }

  std::map<std::string, std::shared_ptr<confusion::Pose<double>>>* getTagReferenceFrameOffsetsMapPointer() { return &referenceFrameOffsets_; }

  ros::NodeHandle &node_;
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber subImage_;
  ros::Subscriber subTagArray_;
  ros::Subscriber subCameraCalibration_;

  image_transport::Publisher imagePub_;
  image_transport::Publisher tagImagePub_;
  ros::Publisher pubTagMarkers_;
  ros::Publisher pubTagArray_;

  ConFusor* conFusorPtr_;

  AprilTagDetector tagDetector_;
  AprilTagParameters aprilTagParameters_;

  // These maps do the bookkeeping on the various frame offsets for the fusion problem
  //todo These should be private with thread-safe getters for asynchronous queries during optimization. Or restrict that they can't be queried during optimization.
  FrameOffsetMap referenceFrameOffsets_; // Vector of offsets from external reference frames to the estimator world frame
  FrameOffsetMap sensorFrameOffsets_;    // Vector of offsets from the IMU to other sensor frames

  // This can be used to indicate when new tag measurements have been added to the ConFusor to tell ongoing optimizations to stop early
  bool *newMeasReceivedFlag_ = nullptr;

private:
  std::string confusionPath_;
  std::string cameraTopic_;
  std::string tagArrayTopic_;
  int tagMeasIndex_;
  bool tagDetectionRunning_ = false;
  bool verbose_ = false;
  double maxImageDt_ = 0.1; //[Hz]
  double t_last_image_ = 0.0;
};

} //namespace confusion

#endif /* INCLUDE_CONFUSION_EXAMPLES_APRILTAGINTERFACE_H_ */
