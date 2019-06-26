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

#include "confusion/modules/apriltag/AprilTagModule.h"

namespace confusion {

AprilTagModule::AprilTagModule(ros::NodeHandle &node,
                  ConFusor* conFusorPtr,
                  std::string configFileName,
                  int tagMeasIndex,
                  bool* newMeasReceivedFlag) :
          node_(node), imageTransport_(node), tagDetector_("tag36h11", 2),
          conFusorPtr_(conFusorPtr), tagMeasIndex_(tagMeasIndex) {
  if (newMeasReceivedFlag)
    newMeasReceivedFlag_ = newMeasReceivedFlag;

  // Load config parameters from the desired file
  confusionPath_ = ros::package::getPath("confusion");
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFileName, pt);

  aprilTagParameters_.tagMeasCalibration_.ftr_init_stddev_ = pt.get<double>("ftr_init_stddev");
  aprilTagParameters_.t_c_i_init_stddev = pt.get<double>("tci_init_stddev");
  aprilTagParameters_.q_c_i_init_stddev = pt.get<double>("qci_init_stddev");
  aprilTagParameters_.tagMeasCalibration_.tagSize_ = pt.get<double>("tagSize");
  aprilTagParameters_.tag_corner_stddev_ = pt.get<double>("tag_corner_stddev");
  aprilTagParameters_.initialize();
  maxImageDt_ = 1.0 / pt.get<double>("maxImageRate");

  //Read the tag map from the text file if desired
  if (pt.get<bool>("getTagMapFromFile")) {
    confusion::Pose<double> T_b_ib;
    confusion::Pose<double> T_ie_e;
    Eigen::Matrix<double, 4, 1> b_q;
    std::string tagMapFilename =
        confusionPath_ + pt.get<std::string>("tagMapFilename");

    if (readTagPoses(tagMapFilename, referenceFrameOffsets_)) {
      //Add the tag poses to the static parameters
      Eigen::MatrixXd ftr_initialWeighting(2, 2);
      ftr_initialWeighting.setIdentity();
      ftr_initialWeighting /= pt.get<double>("ftr_prior_stddev");
      Eigen::MatrixXd t_w_t_initialWeighting(3, 3);
      t_w_t_initialWeighting.setIdentity();
      t_w_t_initialWeighting /= pt.get<double>("twt_prior_stddev");
      Eigen::MatrixXd q_w_t_initialWeighting(3, 3);
      q_w_t_initialWeighting.setIdentity();
      q_w_t_initialWeighting /= pt.get<double>("qwt_prior_stddev");

      //Initialize the first tag, which is either completely fixed or has two rotational degrees of freedom
#ifdef OPT_GRAVITY // *** Optionally defined in the ImuState header file
      conFusorPtr_->staticParameters_.addParameter(confusion::Parameter(
              referenceFrameOffsets_.begin()->second->trans.data(), 3, referenceFrameOffsets_.begin()->first + "_trans", true));
      conFusorPtr_->staticParameters_.addParameter(confusion::Parameter(
              referenceFrameOffsets_.begin()->second->rot.coeffs().data(), 4, referenceFrameOffsets_.begin()->first + "_rot", true,
              std::make_shared<confusion::QuatParam>()));
#else
      conFusorPtr_->staticParameters_.addParameter(confusion::Parameter(
          referenceFrameOffsets_.begin()->second->trans.data(),
          3,
          referenceFrameOffsets_.begin()->first + "_trans",
          true));
      confusion::Parameter ftrParam(
          referenceFrameOffsets_.begin()->second->rot.coeffs().data(), 4,
          referenceFrameOffsets_.begin()->first + "_rot",
          std::make_shared<confusion::FixedYawParameterization>());
      ftrParam.setInitialConstraintWeighting(ftr_initialWeighting);
      conFusorPtr_->staticParameters_.addParameter(ftrParam);
#endif

      //Initialize all other reference frames
      for (auto externalReferenceFrame: referenceFrameOffsets_) {
        confusion::Parameter
            tagTransParam(externalReferenceFrame.second->trans.data(), 3,
                          externalReferenceFrame.first + "_trans");
        tagTransParam.setInitialConstraintWeighting(t_w_t_initialWeighting);
        confusion::Parameter tagRotParam(
            externalReferenceFrame.second->rot.coeffs().data(),
            4,
            externalReferenceFrame.first + "_rot",
            std::make_shared<confusion::QuatParam>());
        tagRotParam.setInitialConstraintWeighting(q_w_t_initialWeighting);
        conFusorPtr_->staticParameters_.addParameter(tagTransParam);
        conFusorPtr_->staticParameters_.addParameter(tagRotParam);
      }
    }
  }

  //Initialize camera/imu offset
  auto T_c_i_ptr = std::make_shared<confusion::Pose<double>>();
  T_c_i_ptr->trans(0) = pt.get<double>("T_c_i.px");
  T_c_i_ptr->trans(1) = pt.get<double>("T_c_i.py");
  T_c_i_ptr->trans(2) = pt.get<double>("T_c_i.pz");
  T_c_i_ptr->rot.w() = pt.get<double>("T_c_i.qw");
  T_c_i_ptr->rot.x() = pt.get<double>("T_c_i.qx");
  T_c_i_ptr->rot.y() = pt.get<double>("T_c_i.qy");
  T_c_i_ptr->rot.z() = pt.get<double>("T_c_i.qz");
  T_c_i_ptr->rot.normalize();
  sensorFrameOffsets_["cam"] = T_c_i_ptr;

  if (pt.get<bool>("optimizeTci")) {
    //Optimize Tci
    Eigen::MatrixXd t_c_i_initialWeighting(3, 3);
    t_c_i_initialWeighting.setIdentity();
    t_c_i_initialWeighting /= aprilTagParameters_.t_c_i_init_stddev;
    confusion::Parameter tciParam(T_c_i_ptr->trans.data(), 3, "t_c_i");
    tciParam.setInitialConstraintWeighting(t_c_i_initialWeighting);
    conFusorPtr_->addStaticParameter(tciParam);
    Eigen::MatrixXd q_c_i_initialWeighting(3, 3);
    q_c_i_initialWeighting.setIdentity();
    q_c_i_initialWeighting /= aprilTagParameters_.q_c_i_init_stddev;
    confusion::Parameter qciParam(T_c_i_ptr->rot.coeffs().data(), 4, "q_c_i",
                                  std::make_shared<confusion::QuatParam>());
    qciParam.setInitialConstraintWeighting(q_c_i_initialWeighting);
    conFusorPtr_->addStaticParameter(qciParam);
  } else {
    //Set up Tci parameters as constant
    conFusorPtr_->addStaticParameter(
        confusion::Parameter(T_c_i_ptr->trans.data(), 3, "T_c_i", true));
    conFusorPtr_->addStaticParameter(
        confusion::Parameter(T_c_i_ptr->rot.coeffs().data(),
                             4,
                             "q_c_i",
                             true,
                             std::make_shared<confusion::QuatParam>()));
  }

  pubTagMarkers_ = node_.advertise<visualization_msgs::MarkerArray>(
      "/tag_markers", 10);
  tagImagePub_ = imageTransport_.advertise("/detector_image", 1);
  pubTagArray_ = node_.advertise<confusion::TagArray>("/tags_detected", 10);

  //Start listening for the camera info message
  subCameraCalibration_ = node_.subscribe(
      pt.get<std::string>("camera_calibration_topic"), 10,
      &AprilTagModule::camCalCallback, this);
  cameraTopic_ = pt.get<std::string>("camera_topic");
  tagArrayTopic_ = pt.get<std::string>("tag_array_topic");

  std::cout << "[AprilTagModule] Initialization complete. Waiting for the cmaera calibration." << std::endl;
}

AprilTagModule::~AprilTagModule() {
  //Write tag poses to text file
  if (referenceFrameOffsets_.size() > 0) {
    std::string tagMapFilename = confusionPath_ + "/tag_tracker_map.txt";
    writeTagPoses(tagMapFilename, referenceFrameOffsets_);
  }
}

void AprilTagModule::camCalCallback(
    const sensor_msgs::CameraInfo::ConstPtr &msg) {
  subCameraCalibration_.shutdown();

//  if (firstMeasurement_) {
//    t_epoch_ = msg->header.stamp.toSec();
//    firstMeasurement_ = false;
//  }

  //Set the projection matrix
  aprilTagParameters_.tagMeasCalibration_.projMat_.setZero();
  aprilTagParameters_.tagMeasCalibration_.projMat_(0, 0) = msg->P[0];
  aprilTagParameters_.tagMeasCalibration_.projMat_(0, 2) = msg->P[2];
  aprilTagParameters_.tagMeasCalibration_.projMat_(1, 1) = msg->P[5];
  aprilTagParameters_.tagMeasCalibration_.projMat_(1, 2) = msg->P[6];
  aprilTagParameters_.tagMeasCalibration_.projMat_(2, 2) = 1.0;
  std::cout << "projMat:\n"
            << aprilTagParameters_.tagMeasCalibration_.projMat_ << std::endl;

  //Start listening for camera measurements
  subImage_ = imageTransport_.subscribe(cameraTopic_, 2,
                                        &AprilTagModule::imageCallback,
                                        this);
  subTagArray_ = node_.subscribe<confusion::TagArray>(
      tagArrayTopic_,
      2,
      &AprilTagModule::tagArrayCallback,
      this,
      ros::TransportHints().tcpNoDelay());

  std::cout << "Camera info received. Now listening for camera measurements."
            << std::endl;
}

void AprilTagModule::processTagDetections(double t, std::vector<TagDetection> &tagDetections) {
  //Get the camera/imu offset pose
  std::shared_ptr<confusion::Pose<double>> T_c_i_ptr;
  try {
    T_c_i_ptr = sensorFrameOffsets_["cam"];
  }
  catch (const std::out_of_range &oor) {
    std::cerr
        << "ERROR: T_c_i is not initialized! Throwing out the tag measurements."
        << std::endl;
    return;
  }

  bool addedTagMeas = false;
  for (int i = 0; i < tagDetections.size(); ++i) {
    std::array<std::array<double, 2>, 4> corners;
    //todo The example bag file uses the old ordering of tag corners
//    corners[3][0] = tagDetections[i].corners[0][0];
//    corners[3][1] = tagDetections[i].corners[0][1];
//    corners[2][0] = tagDetections[i].corners[1][0];
//    corners[2][1] = tagDetections[i].corners[1][1];
//    corners[0][0] = tagDetections[i].corners[2][0];
//    corners[0][1] = tagDetections[i].corners[2][1];
//    corners[1][0] = tagDetections[i].corners[3][0];
//    corners[1][1] = tagDetections[i].corners[3][1];
    corners[0][0] = tagDetections[i].corners[0][0];
    corners[0][1] = tagDetections[i].corners[0][1];
    corners[1][0] = tagDetections[i].corners[1][0];
    corners[1][1] = tagDetections[i].corners[1][1];
    corners[2][0] = tagDetections[i].corners[2][0];
    corners[2][1] = tagDetections[i].corners[2][1];
    corners[3][0] = tagDetections[i].corners[3][0];
    corners[3][1] = tagDetections[i].corners[3][1];

    std::string referenceFrameName =
        "tag" + std::to_string(tagDetections[i].id);
    conFusorPtr_->addUpdateMeasurement(std::make_shared<confusion::TagMeas>(
        t, referenceFrameName, *T_c_i_ptr, corners,
        aprilTagParameters_.tagMeasCalibration_, tagMeasIndex_));

    addedTagMeas = true;
  }

  if (newMeasReceivedFlag_)
    *newMeasReceivedFlag_ = addedTagMeas; // To tell the estimator a new measurement is ready

  if (verbose_) {
    std::cout << "Added " << tagDetections.size() << " tag measurements with stamp " << t << std::endl;
//	std::cout << "Latest IMU measurement is at " << t_imu_source-t_epoch << std::endl;
  }
}

void AprilTagModule::tagArrayCallback(const confusion::TagArray::ConstPtr &msg) {
  std::cout << "tagArrayCallback" << std::endl;
  if (!msg->tags.empty()) {
    std::vector<TagDetection> tagDetections;
    for (int i = 0; i < (msg->tags).size(); ++i) {
      tagDetections.push_back(TagDetection(msg->tags[i]));
    }
    processTagDetections(msg->header.stamp.toSec(), tagDetections);
  }
}

void AprilTagModule::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  // Only process images at the desired rate
  if (msg->header.stamp.toSec() - t_last_image_ < maxImageDt_) {
    return;
  }
  t_last_image_ = msg->header.stamp.toSec();

  //Detect tags in the image
  if (tagDetectionRunning_) {
    std::cout << "WARNING: New image received even though tag detection is running. Dropping the image." << std::endl;
    return;
  }

  tagDetectionRunning_ = true;
  std::thread tagDetectionThread(&AprilTagModule::tagDetection, this, msg);
  tagDetectionThread.detach();
}

void AprilTagModule::tagDetection(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("AprilTagModule::imgaeCallback -- cv_bridge exception: %s",
              e.what());
    return;
  }

  std::vector<TagDetection> tagDetections;
  tagDetector_.detectTags(cv_ptr->image, tagDetections);

  //Publish the image with the tags drawn on them
  cv_bridge::CvImagePtr out_cv_img(new cv_bridge::CvImage);
  out_cv_img->encoding = "rgb8";
  out_cv_img->header = msg->header;
  cv::cvtColor(cv_ptr->image, out_cv_img->image, CV_GRAY2RGB);
  tagDetector_.drawTags(out_cv_img->image, tagDetections);
  tagImagePub_.publish(out_cv_img->toImageMsg());

  if (!tagDetections.empty()) {
    //Publish a tagArray message for use in post-processing
    confusion::TagArray tagArrayMsg = getTagArrayMessage(msg->header.stamp,
                                                         tagDetections);
    pubTagArray_.publish(tagArrayMsg);

    //Process the tag detections
    processTagDetections(msg->header.stamp.toSec(), tagDetections);
  }
  tagDetectionRunning_ = false;
}

void AprilTagModule::publishTagMarkers() {
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/world";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.01;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  int i = 0;
  for (auto &externalReferenceFrame: referenceFrameOffsets_) {
    marker.id = i;
    marker.pose = getMsg(*externalReferenceFrame.second);
    markerArray.markers.push_back(marker);
    ++i;
  }
  pubTagMarkers_.publish(markerArray);
}

} // namespace confusion
