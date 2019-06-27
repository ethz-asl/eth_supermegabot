/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "visensor.hpp"

#define THRESHOLD_DATA_DELAY_WARNING 0.1 // in seconds

namespace visensor {
ViSensor::ViSensor(ros::NodeHandle& nh, std::string sensor_ip,
                   const std::map<SensorId::SensorId, int>& slot_ids,
                   const std::map<SensorId::SensorId, int>& is_flipped,
                   const std::map<SensorId::SensorId, visensor::ViCameraLensModel::LensModelTypes>& lens_types,
                   const std::map<SensorId::SensorId, visensor::ViCameraProjectionModel::ProjectionModelTypes>& projection_types,
                   bool use_time_sync)
  : ViSensor(nh, sensor_ip, slot_ids, is_flipped, lens_types, projection_types,
             SensorId::CAM0, SensorId::CAM1, false, use_time_sync) {
}

ViSensor::ViSensor(ros::NodeHandle& nh, std::string sensor_ip,
                   const std::map<SensorId::SensorId, int>& slot_ids,
                   const std::map<SensorId::SensorId, int>& is_flipped,
                   const std::map<SensorId::SensorId, visensor::ViCameraLensModel::LensModelTypes>& lens_types,
                   const std::map<SensorId::SensorId, visensor::ViCameraProjectionModel::ProjectionModelTypes>& projection_types,
                   const SensorId::SensorId& stereo_left_cam, const SensorId::SensorId& stereo_right_cam,
                   bool stereo_flip_disable, bool use_time_sync)
                   : nh_(nh),
                   use_time_sync_(use_time_sync),
                   stereo_left_cam_(stereo_left_cam),
                   stereo_right_cam_(stereo_right_cam) {
  init(sensor_ip, slot_ids, is_flipped, lens_types, projection_types, stereo_flip_disable);
}

ViSensor::~ViSensor() {
}

void ViSensor::init(const std::string& sensor_ip, const std::map<SensorId::SensorId, int>& slot_ids,
                    const std::map<SensorId::SensorId, int>& is_flipped,
                    const std::map<SensorId::SensorId, visensor::ViCameraLensModel::LensModelTypes>& lens_types,
                    const std::map<SensorId::SensorId, visensor::ViCameraProjectionModel::ProjectionModelTypes>& projection_types,
                    bool stereo_flip_disable) {
  try {
    // ip not specified, use autodiscovery to find sensor
    if (sensor_ip == "0.0.0.0")
      drv_.init();
    else
      drv_.init(sensor_ip);
  } catch (visensor::exceptions const &ex) {
    ROS_ERROR("Failed to initialize Sensor: %s", ex.what());
    exit(1);
  }

#ifndef EXPERT_MODE
  // swap left/right camera topic names if cameras are flipped
  if (drv_.isStereoCameraFlipped()){
    const std::map<SensorId::SensorId, std::string>::iterator cam0 = ROS_CAMERA_NAMES.find(stereo_left_cam_);
    const std::map<SensorId::SensorId, std::string>::iterator cam1 = ROS_CAMERA_NAMES.find(stereo_right_cam_);
    if ((cam0 != ROS_CAMERA_NAMES.end()) && (cam1 != ROS_CAMERA_NAMES.end()))
        std::swap(cam0->second, cam1->second);
  }
#endif

  list_of_available_sensors_ = drv_.getListOfSensorIDs();
  list_of_camera_ids_ = drv_.getListOfCameraIDs();
  list_of_dense_ids_ = drv_.getListOfDenseIDs();
  list_of_imu_ids_ = drv_.getListOfImuIDs();
  list_of_trigger_ids_ = drv_.getListOfTriggerIDs();

  std::string rootdir = ros::package::getPath("visensor_node");
  std::string tempCameraInfoFileName;

  pub_time_host_ = nh_.advertise<visensor_msgs::visensor_time_host>("time_host", -1);

  try {
    drv_.setCameraCallback(boost::bind(&ViSensor::frameCallback, this, _1, _2));
    drv_.setDenseMatcherCallback(boost::bind(&ViSensor::denseCallback, this, _1, _2));
    drv_.setImuCallback(boost::bind(&ViSensor::imuCallback, this, _1, _2));
    drv_.setFramesCornersCallback(boost::bind(&ViSensor::frameCornerCallback, this, _1, _2));
    drv_.setExternalTriggerCallback(boost::bind(&ViSensor::triggerCallback, this, _1));
  } catch (visensor::exceptions const &ex) {
    ROS_WARN("%s", ex.what());
  }

  // initialize cameras
  for (auto camera_id : list_of_camera_ids_) {
    bool calibration_loaded = false;
    try {
      if (slot_ids.count(camera_id)) {
        drv_.selectCameraCalibration(camera_id, slot_ids.at(camera_id), is_flipped.at(camera_id),
                                     lens_types.at(camera_id), projection_types.at(camera_id));  // set camera calibration under the assumtion there is exactly one. Otherwise the exact calibration has specified.
      } else {
        drv_.selectCameraCalibration(camera_id, -1, -1,
                                     ViCameraLensModel::LensModelTypes::UNKNOWN,
                                     ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
      }
      calibration_loaded = true;
      ROS_INFO("[visensor init] Calibration loaded successfully.");
    } catch (visensor::exceptions const &ex) {
      ROS_WARN("[visensor init] Failed to select calibration: %s\nWill try factory equidistant.", ex.what());
    }

    // If we still haven't gotten a calibration, try some fallbacks...
    if (!calibration_loaded) {
      try {
        // Prefer the factory equidistant calibration.
        drv_.selectCameraCalibration(camera_id, 0, -1,
                                     ViCameraLensModel::LensModelTypes::EQUIDISTANT,
                                     ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
        ROS_INFO("[visensor init] Backup factory equidistant calibration loaded successfully.");
        calibration_loaded = true;
      }  catch (visensor::exceptions const &ex) {
        ROS_WARN("[visensor init] Failed to load backup calibration, running without a calibration.", ex.what());
      }
    }

    ros::NodeHandle nhc_temp(nh_, ROS_CAMERA_NAMES.at(camera_id));
    nhc_.insert(std::pair<SensorId::SensorId, ros::NodeHandle>(camera_id, nhc_temp));
    image_transport::ImageTransport itc_temp(nhc_[camera_id]);
    itc_.insert(std::pair<SensorId::SensorId, image_transport::ImageTransport>(camera_id, itc_temp));
    ROS_INFO("Register publisher for camera %u with topic name %s", camera_id,
             ROS_CAMERA_NAMES.at(camera_id).c_str());
    image_transport::CameraPublisher image_pub = (itc_.at(camera_id)).advertiseCamera("image_raw", 100);
    image_pub_.insert(std::pair<SensorId::SensorId, image_transport::CameraPublisher>(camera_id, image_pub));

    precacheViCalibration(camera_id);

    sensor_msgs::CameraInfo cinfo_temp;
    if (getRosCameraConfig(camera_id, cinfo_temp))
      ROS_INFO_STREAM("Read calibration for "<< ROS_CAMERA_NAMES.at(camera_id));
    else
      ROS_INFO_STREAM("Could not read calibration for "<< ROS_CAMERA_NAMES.at(camera_id));

    cinfo_.insert(std::pair<SensorId::SensorId, sensor_msgs::CameraInfo>(camera_id, cinfo_temp));

    ros::Publisher temp_pub;
    temp_pub = nhc_temp.advertise<visensor_msgs::visensor_calibration>("calibration", 1, true);
    calibration_pub_.insert(std::pair<visensor::SensorId::SensorId, ros::Publisher>(camera_id, temp_pub));
  }

  //Generate Stereo ROS config, assuming than cam0 and cam1 are in fronto-parallel stereo configuration
  if(std::find(list_of_camera_ids_.begin(), list_of_camera_ids_.end(), stereo_left_cam_)!=list_of_camera_ids_.end()
      && std::find(list_of_camera_ids_.begin(), list_of_camera_ids_.end(), stereo_right_cam_)!=list_of_camera_ids_.end()) {
    if (!drv_.isStereoCameraFlipped() || stereo_flip_disable){
      //Generate Stereo ROS config, assuming than cam0 and cam1 are in fronto-parallel stereo configuration
      if (getRosStereoCameraConfig(stereo_left_cam_, cinfo_.at(stereo_left_cam_),
                                   stereo_right_cam_, cinfo_.at(stereo_right_cam_)))
        ROS_INFO("Generated ROS Stereo Calibration, assuming %s (left) and %s (right) are a stereo pair.",
                 ROS_CAMERA_NAMES[stereo_left_cam_].c_str(), ROS_CAMERA_NAMES[stereo_right_cam_].c_str());
      else
        ROS_WARN("Could not read stereo calibration for %s and %s.",
                 ROS_CAMERA_NAMES[stereo_left_cam_].c_str(), ROS_CAMERA_NAMES[stereo_right_cam_].c_str());
    } else {
      //Generate Stereo ROS config, assuming than cam0 and cam1 are in fronto-parallel stereo configuration
      if (getRosStereoCameraConfig(stereo_right_cam_, cinfo_.at(stereo_right_cam_),
                                   stereo_left_cam_, cinfo_.at(stereo_left_cam_)))
        ROS_INFO("Generated ROS Stereo Calibration, assuming %s (left) and %s (right) are a stereo pair.",
                 ROS_CAMERA_NAMES[stereo_right_cam_].c_str(), ROS_CAMERA_NAMES[stereo_left_cam_].c_str());
      else
        ROS_WARN("Could not read stereo calibration for %s and %s.",
                 ROS_CAMERA_NAMES[stereo_right_cam_].c_str(), ROS_CAMERA_NAMES[stereo_left_cam_].c_str());
    }
  }

  // initialize dense cameras
  for (auto dense_id : list_of_dense_ids_) {
    ros::NodeHandle nhc_temp(nh_, "dense");
    nhc_.insert(std::pair<SensorId::SensorId, ros::NodeHandle>(dense_id, nhc_temp));
    image_transport::ImageTransport itc_temp(nhc_[dense_id]);
    itc_.insert(std::pair<SensorId::SensorId, image_transport::ImageTransport>(dense_id, itc_temp));
    ROS_INFO_STREAM("Register publisher for dense " << dense_id);
    image_transport::CameraPublisher image_pub = (itc_.at(dense_id)).advertiseCamera("image_raw", 100);
    image_pub_.insert(std::pair<SensorId::SensorId, image_transport::CameraPublisher>(dense_id, image_pub));

    //sensor_msgs::CameraInfo cinfo_temp;
    //if (getRosCameraConfig(dense_id, cinfo_temp))
    //  ROS_INFO_STREAM("Read calibration for "<< dense_id);
    //else
    //  ROS_INFO_STREAM("Could not read calibration for "<< dense_id);

    //TODO(omaris) Adapt camera info messages accordingly
    //cinfo_.insert(std::pair<SensorId::SensorId, sensor_msgs::CameraInfo>(dense_id, cinfo_temp));
  }

  // Initialize imus
  for (auto imu_id : list_of_imu_ids_) {
#ifndef EXPERT_MODE
    if (imu_id != SensorId::IMU0)
      continue;
#endif
    ros::Publisher temp_pub;
    temp_pub = nh_.advertise<sensor_msgs::Imu>(ROS_IMU_NAMES.at(imu_id), 4000);
    printf("register publisher for imu %u\n", imu_id);
    imu_pub_.insert(std::pair<visensor::SensorId::SensorId, ros::Publisher>(imu_id, temp_pub));
    temp_pub = nh_.advertise<visensor_msgs::visensor_imu>("cust_" + ROS_IMU_NAMES.at(imu_id), -1);
    imu_custom_pub_.insert(std::pair<visensor::SensorId::SensorId, ros::Publisher>(imu_id, temp_pub));
  }

  trigger_pub_ = nh_.advertise<visensor_msgs::visensor_trigger>("external_trigger", -1);
  calibration_service_ = nh_.advertiseService("get_camera_calibration", &ViSensor::calibrationServiceCallback, this);

  // init dynamic reconfigure
  dr_srv_.setCallback(boost::bind(&ViSensor::configCallback, this, _1, _2));

  ROS_INFO("Initialize VI-Sensor with Sensor ID %i", drv_.getViSensorId());

}

void ViSensor::startSensors(std::map<SensorId::SensorId, int>& cam_rates,
                            const int com_rate_global, const unsigned int imu_rate,
                            const unsigned int trigger_rate)
{
  for (uint i = 0; i < list_of_camera_ids_.size(); i++) {
    int cam_rate = com_rate_global;
    if ( cam_rates.count(list_of_camera_ids_[i]) ) {
     if (cam_rates.at(list_of_camera_ids_[i]) > 0)
       cam_rate = cam_rates[list_of_camera_ids_[i]];
    }
    drv_.startSensor(list_of_camera_ids_[i], cam_rate);
  }

  drv_.startAllCorners();
#ifdef EXPERT_MODE
  drv_.startAllImus(imu_rate);
#else
  if(drv_.isSensorPresent(visensor::SensorId::IMU0))
    drv_.startSensor(visensor::SensorId::IMU0, imu_rate);
#endif
  drv_.startAllExternalTriggers(trigger_rate);
  drv_.startAllDenseMatchers();
  if(drv_.isSensorPresent(SensorId::LED_FLASHER0))
    drv_.startSensor(visensor::SensorId::LED_FLASHER0);
}

void ViSensor::imuCallback(boost::shared_ptr<ViImuMsg> imu_ptr, ViErrorCode error) {
  if (error == ViErrorCodes::MEASUREMENT_DROPPED) {
    ROS_WARN("dropped imu measurement on sensor %u (check network bandwidth/sensor rate)", imu_ptr->imu_id);
    return;
  }

  const double sigma2_gyr_adis16375_d = 0;
  const double sigma2_acc_adis16375_d = 0;

  ros::Time msg_time;
  if (use_time_sync_)
    msg_time.fromNSec(imu_ptr->timestamp_synchronized);
  else
    msg_time.fromNSec(imu_ptr->timestamp);

  sensor_msgs::Imu imu_msg;

  imu_msg.header.stamp = msg_time;
  imu_msg.header.frame_id = ROS_IMU_FRAME_NAMES.at(static_cast<SensorId::SensorId>(imu_ptr->imu_id));
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance[0] = 99999.9;
  imu_msg.orientation_covariance[1] = 0.0;
  imu_msg.orientation_covariance[2] = 0.0;
  imu_msg.orientation_covariance[3] = 0.0;
  imu_msg.orientation_covariance[4] = 99999.9;
  imu_msg.orientation_covariance[5] = 0.0;
  imu_msg.orientation_covariance[6] = 0.0;
  imu_msg.orientation_covariance[7] = 0.0;
  imu_msg.orientation_covariance[8] = 99999.9;
  // --- Angular Velocity.
  imu_msg.angular_velocity.x = imu_ptr->gyro[0];
  imu_msg.angular_velocity.y = imu_ptr->gyro[1];
  imu_msg.angular_velocity.z = imu_ptr->gyro[2];
  imu_msg.angular_velocity_covariance[0] = sigma2_gyr_adis16375_d;
  imu_msg.angular_velocity_covariance[1] = 0.0;
  imu_msg.angular_velocity_covariance[2] = 0.0;
  imu_msg.angular_velocity_covariance[3] = 0.0;
  imu_msg.angular_velocity_covariance[4] = sigma2_gyr_adis16375_d;
  imu_msg.angular_velocity_covariance[5] = 0.0;
  imu_msg.angular_velocity_covariance[6] = 0.0;
  imu_msg.angular_velocity_covariance[7] = 0.0;
  imu_msg.angular_velocity_covariance[8] = sigma2_gyr_adis16375_d;
  // --- Linear Acceleration.
  imu_msg.linear_acceleration.x = imu_ptr->acc[0];
  imu_msg.linear_acceleration.y = imu_ptr->acc[1];
  imu_msg.linear_acceleration.z = imu_ptr->acc[2];
  imu_msg.linear_acceleration_covariance[0] = sigma2_acc_adis16375_d;
  imu_msg.linear_acceleration_covariance[1] = 0.0;
  imu_msg.linear_acceleration_covariance[2] = 0.0;
  imu_msg.linear_acceleration_covariance[3] = 0.0;
  imu_msg.linear_acceleration_covariance[4] = sigma2_acc_adis16375_d;
  imu_msg.linear_acceleration_covariance[5] = 0.0;
  imu_msg.linear_acceleration_covariance[6] = 0.0;
  imu_msg.linear_acceleration_covariance[7] = 0.0;
  imu_msg.linear_acceleration_covariance[8] = sigma2_acc_adis16375_d;

  visensor_msgs::visensor_imu imu2;
  imu2.header.stamp = msg_time;
  imu2.header.frame_id = ROS_IMU_FRAME_NAMES.at(static_cast<SensorId::SensorId>(imu_ptr->imu_id));
  imu2.header.seq = 5;

  imu2.angular_velocity.x = imu_ptr->gyro[0];
  imu2.angular_velocity.y = imu_ptr->gyro[1];
  imu2.angular_velocity.z = imu_ptr->gyro[2];

  imu2.linear_acceleration.x = imu_ptr->acc[0];
  imu2.linear_acceleration.y = imu_ptr->acc[1];
  imu2.linear_acceleration.z = imu_ptr->acc[2];

  imu2.magnetometer.x = imu_ptr->mag[0];
  imu2.magnetometer.y = imu_ptr->mag[1];
  imu2.magnetometer.z = imu_ptr->mag[2];

  imu2.pressure = imu_ptr->baro;
  imu2.temperature = imu_ptr->temperature;

  // --- Publish IMU Message.
  imu_pub_.at(static_cast<SensorId::SensorId>(imu_ptr->imu_id)).publish(imu_msg);

  // --- Publish custom IMU Message.
  imu_custom_pub_.at(static_cast<SensorId::SensorId>(imu_ptr->imu_id)).publish(imu2);
}

void ViSensor::frameCallback(ViFrame::Ptr frame_ptr, ViErrorCode error) {
  if (error == ViErrorCodes::MEASUREMENT_DROPPED) {
    ROS_WARN("dropped camera image on sensor %u (check network bandwidth/sensor rate)",
             frame_ptr->camera_id);
    return;
  }

  int image_height = frame_ptr->height;
  int image_width = frame_ptr->width;

  // get sensor time of message
  ros::Time msg_time;
  if (use_time_sync_)
    msg_time.fromNSec(frame_ptr->timestamp_synchronized);
  else
    msg_time.fromNSec(frame_ptr->timestamp);

  // check if transmission is delayed
  const double frame_delay = (ros::Time::now() - msg_time).toSec();
  if (frame_delay > THRESHOLD_DATA_DELAY_WARNING)
    ROS_WARN("Data arrived later than expected [ms]: %f", frame_delay * 1000.0);

  // get system time of message
  ros::Time msg_time_host;
  msg_time_host.fromNSec(frame_ptr->timestamp_host);

  // create new time message
  visensor_msgs::visensor_time_host time_msg;
  time_msg.header.stamp = msg_time;
  time_msg.timestamp_host = msg_time_host;
  pub_time_host_.publish(time_msg);

  // create new image message
  sensor_msgs::Image msg;
  msg.header.stamp = msg_time;
  if (frame_ptr->camera_id == stereo_right_cam_) {
    msg.header.frame_id = ROS_CAMERA_FRAME_NAMES.at(static_cast<SensorId::SensorId>(stereo_left_cam_));
  } else {
    msg.header.frame_id = ROS_CAMERA_FRAME_NAMES.at(static_cast<SensorId::SensorId>(frame_ptr->camera_id));
  }

  if (frame_ptr->image_type == MONO8)
    sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8, image_height, image_width, image_width,
                           frame_ptr->getImageRawPtr());
  else if (frame_ptr->image_type == MONO16) {
    cv::Mat image;
    image.create(image_height, image_width, CV_16UC1);

    cv::Mat image_8bit;
    image_8bit.create(image_height, image_width, CV_8UC1);

    memcpy(image.data, frame_ptr->getImageRawPtr(), (image_width) * image_height * 2);

    sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO16, image_height, image_width, image_width * 2,
                           image.data);
  } else
    ROS_WARN("[VI_SENSOR] - unknown image type!");

  // get current CameraInfo data
  sensor_msgs::CameraInfo ci = cinfo_[static_cast<SensorId::SensorId>(frame_ptr->camera_id)];

  // fill header
  if (frame_ptr->camera_id == stereo_right_cam_) {
    ci.header.frame_id = ROS_CAMERA_FRAME_NAMES.at(static_cast<SensorId::SensorId>(stereo_left_cam_));
  } else {
    ci.header.frame_id = ROS_CAMERA_FRAME_NAMES.at(static_cast<SensorId::SensorId>(frame_ptr->camera_id));
  }
  ci.header.stamp = msg_time;

  ci.height = image_height;
  ci.width = image_width;

  // publish image
  image_pub_[static_cast<SensorId::SensorId>(frame_ptr->camera_id)].publish(msg, ci);

  calibration_pub_[static_cast<SensorId::SensorId>(frame_ptr->camera_id)].publish(
      camera_imu_calibrations_[ROS_CAMERA_NAMES.at(static_cast<SensorId::SensorId>(frame_ptr->camera_id))]);

  if (camera_imu_transformations_.find(ROS_CAMERA_NAMES.at(static_cast<SensorId::SensorId>(frame_ptr->camera_id)))
      != camera_imu_transformations_.end()) {
    br_.sendTransform(
        tf::StampedTransform(
            camera_imu_transformations_[ROS_CAMERA_NAMES.at(static_cast<SensorId::SensorId>(frame_ptr->camera_id))],
            msg_time, ROS_IMU_NAMES.at(SensorId::IMU0), ROS_CAMERA_NAMES.at(static_cast<SensorId::SensorId>(frame_ptr->camera_id))));
  }
}

void ViSensor::denseCallback(ViFrame::Ptr frame_ptr, ViErrorCode error) {
  if (error == ViErrorCodes::MEASUREMENT_DROPPED) {
    ROS_WARN("dropped dense image on sensor %u (check network bandwidth/sensor rate)", frame_ptr->camera_id);
    return;
  }

  int image_height = frame_ptr->height;
  int image_width = frame_ptr->width;

  // get sensor time of message
  ros::Time msg_time;
  if (use_time_sync_)
    msg_time.fromNSec(frame_ptr->timestamp_synchronized);
  else
    msg_time.fromNSec(frame_ptr->timestamp);

  // get system time of message
  ros::Time msg_time_host;
  msg_time_host.fromNSec(frame_ptr->timestamp_host);

  // create new time message
  visensor_msgs::visensor_time_host time_msg;
  time_msg.header.stamp = msg_time;
  time_msg.timestamp_host = msg_time_host;
  pub_time_host_.publish(time_msg);

  // create new image message
  sensor_msgs::Image msg;
  msg.header.stamp = msg_time;
  msg.header.frame_id = "dense";

  sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8, image_height, image_width, image_width,
                         frame_ptr->getImageRawPtr());
  sensor_msgs::CameraInfo ci = cinfo_[static_cast<SensorId::SensorId>(frame_ptr->camera_id)];

  // fill header
  ci.header.frame_id = "dense";
  ci.header.stamp = msg_time;

  ci.height = image_height;
  ci.width = image_width;

  // publish image
  image_pub_[static_cast<SensorId::SensorId>(frame_ptr->camera_id)].publish(msg, ci);

}

void ViSensor::frameCornerCallback(ViFrame::Ptr frame_ptr, ViCorner::Ptr corners_ptr) {
  int image_height = frame_ptr->height;
  int image_width = frame_ptr->width;

  cv::Mat image_8bit;
  image_8bit.create(image_height, image_width, CV_8UC1);
  memcpy(image_8bit.data, frame_ptr->getImageRawPtr(), image_width * image_height);

  cv::Mat image_color;
  image_color.create(image_height, image_width, CV_8UC3);

  cv::cvtColor(image_8bit, image_color, CV_GRAY2BGR);
  std::vector<cv::KeyPoint> keypoints;

  for (uint32_t i = 0; i < corners_ptr->corners.size(); ++i) {

    const ViCornerElem& current_corner = corners_ptr->corners.at(i);

    if (current_corner.score < 1 << 22)
      continue;
    keypoints.push_back(
        cv::KeyPoint((float) current_corner.x, (float) current_corner.y, 12, -1, (float) current_corner.score));
  }

  cv::Mat image_out;
  image_color.create(image_height, image_width, CV_8UC3);

  cv::drawKeypoints(image_color, keypoints, image_out);
  cv::imshow("Detected Harris corners", image_out);
  cv::waitKey(3);
}

void ViSensor::triggerCallback(ViExternalTriggerMsg::Ptr trigger_ptr) {
  visensor_msgs::visensor_trigger msg;

  ros::Time temp_time;
  if (use_time_sync_)
    temp_time.fromNSec(trigger_ptr->timestamp_synchronized);
  else
    temp_time.fromNSec(trigger_ptr->timestamp);

  msg.header.stamp = temp_time;
  msg.trigger_id = trigger_ptr->trigger_id;

  trigger_pub_.publish(msg);
}

void ViSensor::configCallback(visensor_node::DriverConfig& config, uint32_t level) {
  bool individual_cam_config = false;
    const double analog_gain_to_fpga_units = 16.0;
    const double exposure_ms_to_fpga_units = 25.0e6/(1.0e3*(752.0+94.0));

#ifdef EXPERT_MODE
  individual_cam_config = config.individual_cam_config;

  if(individual_cam_config) {
    // configure MPU 9150 IMU (if available)
    if (drv_.isSensorPresent(visensor::SensorId::IMU_CAM0))
      drv_.setSensorConfigParam(visensor::SensorId::IMU_CAM0, "digital_low_pass_filter_config", 0);

    if (drv_.isSensorPresent(visensor::SensorId::IMU_CAM1))
      drv_.setSensorConfigParam(visensor::SensorId::IMU_CAM1, "digital_low_pass_filter_config", 0);

    // ========================= LED Control ==========================
    if (drv_.isSensorPresent(visensor::SensorId::LED_FLASHER0)) {
      drv_.setSensorConfigParam(visensor::SensorId::LED_FLASHER0, "strobe", config.strobe);
      drv_.setSensorConfigParam(visensor::SensorId::LED_FLASHER0, "strobe_mode", config.strobe_mode);
    }

    // ========================= CAMERA 0 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM0)) {
      const int cam0_max_analog_gain = std::round(config.cam0_max_analog_gain*analog_gain_to_fpga_units);
      const int cam0_global_analog_gain = std::round(config.cam0_global_analog_gain*analog_gain_to_fpga_units);
      const int cam0_min_coarse_shutter_width = std::round(config.cam0_min_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam0_max_coarse_shutter_width = std::round(config.cam0_max_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam0_coarse_shutter_width = std::round(config.cam0_coarse_shutter_width*exposure_ms_to_fpga_units);

      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "agc_enable", config.cam0_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "max_analog_gain", cam0_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "global_analog_gain", cam0_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "global_analog_gain_attenuation",
                                config.cam0_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "aec_enable", config.cam0_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "min_coarse_shutter_width",
                                cam0_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "max_coarse_shutter_width",
                                cam0_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "coarse_shutter_width", cam0_coarse_shutter_width);

      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "adc_mode", config.cam0_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "vref_adc_voltage_level",
                                config.cam0_vref_adc_voltage_level);

      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "black_level_calibration_manual_override",
                                config.cam0_black_level_calibration_manual_override);
      if (config.cam0_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM0, "black_level_calibration_value",
                                  config.cam0_black_level_calibration_value);
    }

    // ========================= CAMERA 1 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM1)) {
      const int cam1_max_analog_gain = std::round(config.cam1_max_analog_gain*analog_gain_to_fpga_units);
      const int cam1_global_analog_gain = std::round(config.cam1_global_analog_gain*analog_gain_to_fpga_units);
      const int cam1_min_coarse_shutter_width = std::round(config.cam1_min_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam1_max_coarse_shutter_width = std::round(config.cam1_max_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam1_coarse_shutter_width = std::round(config.cam1_coarse_shutter_width*exposure_ms_to_fpga_units);

      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "agc_enable", config.cam1_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "max_analog_gain", cam1_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "global_analog_gain", cam1_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "global_analog_gain_attenuation",
                                config.cam1_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "aec_enable", config.cam1_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "min_coarse_shutter_width",
                                cam1_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "max_coarse_shutter_width",
                                cam1_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "coarse_shutter_width", cam1_coarse_shutter_width);

      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "adc_mode", config.cam1_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "vref_adc_voltage_level",
                                config.cam1_vref_adc_voltage_level);

      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "black_level_calibration_manual_override",
                                config.cam1_black_level_calibration_manual_override);
      if (config.cam1_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM1, "black_level_calibration_value",
                                  config.cam1_black_level_calibration_value);
    }

    // ========================= CAMERA 2 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM2)) {
      const int cam2_max_analog_gain = std::round(config.cam2_max_analog_gain*analog_gain_to_fpga_units);
      const int cam2_global_analog_gain = std::round(config.cam2_global_analog_gain*analog_gain_to_fpga_units);
      const int cam2_min_coarse_shutter_width = std::round(config.cam2_min_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam2_max_coarse_shutter_width = std::round(config.cam2_max_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam2_coarse_shutter_width = std::round(config.cam2_coarse_shutter_width*exposure_ms_to_fpga_units);

      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "agc_enable", config.cam2_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "max_analog_gain", cam2_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "global_analog_gain", cam2_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "global_analog_gain_attenuation",
                                config.cam2_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "aec_enable", config.cam2_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "min_coarse_shutter_width",
                                cam2_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "max_coarse_shutter_width",
                                cam2_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "coarse_shutter_width", cam2_coarse_shutter_width);

      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "adc_mode", config.cam2_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "vref_adc_voltage_level",
                                config.cam2_vref_adc_voltage_level);

      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "black_level_calibration_manual_override",
                                config.cam2_black_level_calibration_manual_override);
      if (config.cam2_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM2, "black_level_calibration_value",
                                  config.cam2_black_level_calibration_value);
    }

    // ========================= CAMERA 3 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM3)) {
      const int cam3_max_analog_gain = std::round(config.cam3_max_analog_gain*analog_gain_to_fpga_units);
      const int cam3_global_analog_gain = std::round(config.cam3_global_analog_gain*analog_gain_to_fpga_units);
      const int cam3_min_coarse_shutter_width = std::round(config.cam3_min_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam3_max_coarse_shutter_width = std::round(config.cam3_max_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam3_coarse_shutter_width = std::round(config.cam3_coarse_shutter_width*exposure_ms_to_fpga_units);

      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "agc_enable", config.cam3_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "max_analog_gain", cam3_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "global_analog_gain", cam3_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "global_analog_gain_attenuation",
                                config.cam3_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "aec_enable", config.cam3_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "min_coarse_shutter_width",
                                cam3_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "max_coarse_shutter_width",
                                cam3_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "coarse_shutter_width", cam3_coarse_shutter_width);

      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "adc_mode", config.cam3_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "vref_adc_voltage_level",
                                config.cam3_vref_adc_voltage_level);

      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "black_level_calibration_manual_override",
                                config.cam3_black_level_calibration_manual_override);
      if (config.cam3_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM3, "black_level_calibration_value",
                                  config.cam3_black_level_calibration_value);
    }

    // ========================= CAMERA 4 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM4)) {
      const int cam4_max_analog_gain = std::round(config.cam4_max_analog_gain*analog_gain_to_fpga_units);
      const int cam4_global_analog_gain = std::round(config.cam4_global_analog_gain*analog_gain_to_fpga_units);
      const int cam4_min_coarse_shutter_width = std::round(config.cam4_min_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam4_max_coarse_shutter_width = std::round(config.cam4_max_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam4_coarse_shutter_width = std::round(config.cam4_coarse_shutter_width*exposure_ms_to_fpga_units);

      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "agc_enable", config.cam4_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "max_analog_gain", cam4_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "global_analog_gain", cam4_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "global_analog_gain_attenuation",
                                config.cam4_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "aec_enable", config.cam4_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "min_coarse_shutter_width",
                                cam4_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "max_coarse_shutter_width",
                                cam4_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "coarse_shutter_width", cam4_coarse_shutter_width);

      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "adc_mode", config.cam4_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "vref_adc_voltage_level",
                                config.cam4_vref_adc_voltage_level);

      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "black_level_calibration_manual_override",
                                config.cam4_black_level_calibration_manual_override);
      if (config.cam4_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM4, "black_level_calibration_value",
                                  config.cam4_black_level_calibration_value);
    }

    // ========================= CAMERA 5 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM5)) {
      const int cam5_max_analog_gain = std::round(config.cam5_max_analog_gain*analog_gain_to_fpga_units);
      const int cam5_global_analog_gain = std::round(config.cam5_global_analog_gain*analog_gain_to_fpga_units);
      const int cam5_min_coarse_shutter_width = std::round(config.cam5_min_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam5_max_coarse_shutter_width = std::round(config.cam5_max_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam5_coarse_shutter_width = std::round(config.cam5_coarse_shutter_width*exposure_ms_to_fpga_units);

      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "agc_enable", config.cam5_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "max_analog_gain", cam5_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "global_analog_gain", cam5_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "global_analog_gain_attenuation",
                                config.cam5_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "aec_enable", config.cam5_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "min_coarse_shutter_width",
                                cam5_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "max_coarse_shutter_width",
                                cam5_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "coarse_shutter_width", cam5_coarse_shutter_width);

      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "adc_mode", config.cam5_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "vref_adc_voltage_level",
                                config.cam5_vref_adc_voltage_level);

      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "black_level_calibration_manual_override",
                                config.cam5_black_level_calibration_manual_override);
      if (config.cam5_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM5, "black_level_calibration_value",
                                  config.cam5_black_level_calibration_value);
    }

    // ========================= CAMERA 6 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM6)) {
      const int cam6_max_analog_gain = std::round(config.cam6_max_analog_gain*analog_gain_to_fpga_units);
      const int cam6_global_analog_gain = std::round(config.cam6_global_analog_gain*analog_gain_to_fpga_units);
      const int cam6_min_coarse_shutter_width = std::round(config.cam6_min_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam6_max_coarse_shutter_width = std::round(config.cam6_max_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam6_coarse_shutter_width = std::round(config.cam6_coarse_shutter_width*exposure_ms_to_fpga_units);

      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "agc_enable", config.cam6_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "max_analog_gain", cam6_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "global_analog_gain", cam6_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "global_analog_gain_attenuation",
                                config.cam6_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "aec_enable", config.cam6_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "min_coarse_shutter_width",
                                cam6_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "max_coarse_shutter_width",
                                cam6_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "coarse_shutter_width", cam6_coarse_shutter_width);

      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "adc_mode", config.cam6_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "vref_adc_voltage_level",
                                config.cam6_vref_adc_voltage_level);

      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "black_level_calibration_manual_override",
                                config.cam6_black_level_calibration_manual_override);
      if (config.cam6_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM6, "black_level_calibration_value",
                                  config.cam6_black_level_calibration_value);
    }

    // ========================= CAMERA 7 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM7)) {
      const int cam7_max_analog_gain = std::round(config.cam7_max_analog_gain*analog_gain_to_fpga_units);
      const int cam7_global_analog_gain = std::round(config.cam7_global_analog_gain*analog_gain_to_fpga_units);
      const int cam7_min_coarse_shutter_width = std::round(config.cam7_min_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam7_max_coarse_shutter_width = std::round(config.cam7_max_coarse_shutter_width*exposure_ms_to_fpga_units);
      const int cam7_coarse_shutter_width = std::round(config.cam7_coarse_shutter_width*exposure_ms_to_fpga_units);

      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "agc_enable", config.cam7_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "max_analog_gain", cam7_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "global_analog_gain", cam7_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "global_analog_gain_attenuation",
                                config.cam7_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "aec_enable", config.cam7_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "min_coarse_shutter_width",
                                cam7_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "max_coarse_shutter_width",
                                cam7_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "coarse_shutter_width", cam7_coarse_shutter_width);

      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "adc_mode", config.cam7_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "vref_adc_voltage_level",
                                config.cam7_vref_adc_voltage_level);

      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "black_level_calibration_manual_override",
                                config.cam7_black_level_calibration_manual_override);
      if (config.cam7_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM7, "black_level_calibration_value",
                                  config.cam7_black_level_calibration_value);
    }

    // ========================= DENSE MATCHER ==========================
    if (drv_.isSensorPresent(visensor::SensorId::DENSE_MATCHER0)) {
      drv_.setSensorConfigParam(visensor::SensorId::DENSE_MATCHER0, "penalty_1", config.penalty_1);
      drv_.setSensorConfigParam(visensor::SensorId::DENSE_MATCHER0, "penalty_2", config.penalty_2);
      drv_.setSensorConfigParam(visensor::SensorId::DENSE_MATCHER0, "threshold", config.threshold);
      drv_.setSensorConfigParam(visensor::SensorId::DENSE_MATCHER0, "lr_check", config.lr_check);
    }
    // ========================= TRIGGER  ==========================
    if (drv_.isSensorPresent(visensor::SensorId::EXTERNAL_TRIGGER0)) {
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "enable_trigger_0", config.enable_trigger_0);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "enable_trigger_1", config.enable_trigger_1);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "enable_trigger_2", config.enable_trigger_2);

      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_0_direction", config.trigger_0_direction);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_1_direction", config.trigger_1_direction);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_2_direction", config.trigger_2_direction);

      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_0_rate", 100000.0/config.trigger_0_rate);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_1_rate", 100000.0/config.trigger_1_rate);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_2_rate", 100000.0/config.trigger_2_rate);

      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_0_offset", config.trigger_0_offset);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_1_offset", config.trigger_1_offset);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_2_offset", config.trigger_2_offset);
    }
  }

#endif
  if(!individual_cam_config) {
    // configure MPU 9150 IMU (if available)
    if (drv_.isSensorPresent(visensor::SensorId::IMU_CAM0))
      drv_.setSensorConfigParam(visensor::SensorId::IMU_CAM0, "digital_low_pass_filter_config", 0);

    if (drv_.isSensorPresent(visensor::SensorId::IMU_CAM1))
      drv_.setSensorConfigParam(visensor::SensorId::IMU_CAM1, "digital_low_pass_filter_config", 0);

    // ========================= LED Control ==========================
    if (drv_.isSensorPresent(visensor::SensorId::LED_FLASHER0)) {
      drv_.setSensorConfigParam(visensor::SensorId::LED_FLASHER0, "strobe", config.strobe);
      drv_.setSensorConfigParam(visensor::SensorId::LED_FLASHER0, "strobe_mode", config.strobe_mode);
    }

    const int cam_max_analog_gain = std::round(config.cam_max_analog_gain*analog_gain_to_fpga_units);
    const int cam_global_analog_gain = std::round(config.cam_global_analog_gain*analog_gain_to_fpga_units);
    const int cam_min_coarse_shutter_width = std::round(config.cam_min_coarse_shutter_width*exposure_ms_to_fpga_units);
    const int cam_max_coarse_shutter_width = std::round(config.cam_max_coarse_shutter_width*exposure_ms_to_fpga_units);
    const int cam_coarse_shutter_width = std::round(config.cam_coarse_shutter_width*exposure_ms_to_fpga_units);

    // ========================= CAMERA 0 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM0)) {
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "agc_enable", config.cam_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "max_analog_gain", cam_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "global_analog_gain", cam_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "global_analog_gain_attenuation",
                                config.cam_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "aec_enable", config.cam_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "min_coarse_shutter_width",
                                cam_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "max_coarse_shutter_width",
                                cam_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "coarse_shutter_width", cam_coarse_shutter_width);

     /*
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "adc_mode", cam_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "vref_adc_voltage_level",
                                cam_vref_adc_voltage_level);
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "black_level_calibration_manual_override",
                                config.cam_black_level_calibration_manual_override);
      if (config.cam_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM0, "black_level_calibration_value",
                                  config.cam_black_level_calibration_value);*/
    }

    // ========================= CAMERA 1 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM1)) {
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "agc_enable", config.cam_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "max_analog_gain", cam_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "global_analog_gain", cam_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "global_analog_gain_attenuation",
                                config.cam_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "aec_enable", config.cam_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "min_coarse_shutter_width",
                                cam_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "max_coarse_shutter_width",
                                cam_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "coarse_shutter_width", cam_coarse_shutter_width);

     /*
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "adc_mode", config.cam_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "vref_adc_voltage_level",
                                config.cam_vref_adc_voltage_level);
      drv_.setSensorConfigParam(visensor::SensorId::CAM1, "black_level_calibration_manual_override",
                                config.cam_black_level_calibration_manual_override);
      if (config.cam_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM1, "black_level_calibration_value",
                                  config.cam_black_level_calibration_value);*/
    }

    // ========================= CAMERA 2 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM2)) {
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "agc_enable", config.cam_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "max_analog_gain", cam_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "global_analog_gain", cam_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "global_analog_gain_attenuation",
                                config.cam_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "aec_enable", config.cam_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "min_coarse_shutter_width",
                                cam_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "max_coarse_shutter_width",
                                cam_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "coarse_shutter_width", cam_coarse_shutter_width);

     /*
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "adc_mode", config.cam_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "vref_adc_voltage_level",
                                config.cam_vref_adc_voltage_level);
      drv_.setSensorConfigParam(visensor::SensorId::CAM2, "black_level_calibration_manual_override",
                                config.cam_black_level_calibration_manual_override);
      if (config.cam_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM2, "black_level_calibration_value",
                                  config.cam_black_level_calibration_value);
      */
    }

    // ========================= CAMERA 3 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM3)) {
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "agc_enable", config.cam_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "max_analog_gain", cam_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "global_analog_gain", cam_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "global_analog_gain_attenuation",
                                config.cam_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "aec_enable", config.cam_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "min_coarse_shutter_width",
                                cam_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "max_coarse_shutter_width",
                                cam_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "coarse_shutter_width", cam_coarse_shutter_width);

      /*
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "adc_mode", config.cam_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "vref_adc_voltage_level",
                                config.cam_vref_adc_voltage_level);
      drv_.setSensorConfigParam(visensor::SensorId::CAM3, "black_level_calibration_manual_override",
                                config.cam_black_level_calibration_manual_override);
      if (config.cam_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM3, "black_level_calibration_value",
                                  config.cam_black_level_calibration_value);
      */
    }

    // ========================= CAMERA 4 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM4)) {
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "agc_enable", config.cam_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "max_analog_gain", cam_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "global_analog_gain", cam_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "global_analog_gain_attenuation",
                                config.cam_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "aec_enable", config.cam_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "min_coarse_shutter_width",
                                cam_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "max_coarse_shutter_width",
                                cam_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "coarse_shutter_width", cam_coarse_shutter_width);

     /*
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "adc_mode", cam_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "vref_adc_voltage_level",
                                cam_vref_adc_voltage_level);
      drv_.setSensorConfigParam(visensor::SensorId::CAM4, "black_level_calibration_manual_override",
                                config.cam_black_level_calibration_manual_override);
      if (config.cam_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM4, "black_level_calibration_value",
                                  config.cam_black_level_calibration_value);*/
    }

    // ========================= CAMERA 5 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM5)) {
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "agc_enable", config.cam_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "max_analog_gain", cam_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "global_analog_gain", cam_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "global_analog_gain_attenuation",
                                config.cam_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "aec_enable", config.cam_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "min_coarse_shutter_width",
                                cam_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "max_coarse_shutter_width",
                                cam_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "coarse_shutter_width", cam_coarse_shutter_width);

     /*
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "adc_mode", config.cam_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "vref_adc_voltage_level",
                                config.cam_vref_adc_voltage_level);
      drv_.setSensorConfigParam(visensor::SensorId::CAM5, "black_level_calibration_manual_override",
                                config.cam_black_level_calibration_manual_override);
      if (config.cam_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM5, "black_level_calibration_value",
                                  config.cam_black_level_calibration_value);*/
    }

    // ========================= CAMERA 6 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM6)) {
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "agc_enable", config.cam_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "max_analog_gain", cam_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "global_analog_gain", cam_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "global_analog_gain_attenuation",
                                config.cam_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "aec_enable", config.cam_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "min_coarse_shutter_width",
                                cam_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "max_coarse_shutter_width",
                                cam_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "coarse_shutter_width", cam_coarse_shutter_width);

     /*
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "adc_mode", config.cam_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "vref_adc_voltage_level",
                                config.cam_vref_adc_voltage_level);
      drv_.setSensorConfigParam(visensor::SensorId::CAM6, "black_level_calibration_manual_override",
                                config.cam_black_level_calibration_manual_override);
      if (config.cam_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM6, "black_level_calibration_value",
                                  config.cam_black_level_calibration_value);
      */
    }

    // ========================= CAMERA 7 ==========================
    if (drv_.isSensorPresent(visensor::SensorId::CAM7)) {
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "agc_enable", config.cam_agc_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "max_analog_gain", cam_max_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "global_analog_gain", cam_global_analog_gain);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "global_analog_gain_attenuation",
                                config.cam_global_analog_gain_attenuation);

      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "aec_enable", config.cam_aec_enable);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "min_coarse_shutter_width",
                                cam_min_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "max_coarse_shutter_width",
                                cam_max_coarse_shutter_width);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "coarse_shutter_width", cam_coarse_shutter_width);

      /*
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "adc_mode", config.cam_adc_mode);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "vref_adc_voltage_level",
                                config.cam_vref_adc_voltage_level);
      drv_.setSensorConfigParam(visensor::SensorId::CAM7, "black_level_calibration_manual_override",
                                config.cam_black_level_calibration_manual_override);
      if (config.cam_black_level_calibration_manual_override)
        drv_.setSensorConfigParam(visensor::SensorId::CAM7, "black_level_calibration_value",
                                  config.cam_black_level_calibration_value);
      */
    }

    // ========================= DENSE MATCHER ==========================
    if (drv_.isSensorPresent(visensor::SensorId::DENSE_MATCHER0)) {
      drv_.setSensorConfigParam(visensor::SensorId::DENSE_MATCHER0, "penalty_1", config.penalty_1);
      drv_.setSensorConfigParam(visensor::SensorId::DENSE_MATCHER0, "penalty_2", config.penalty_2);
      drv_.setSensorConfigParam(visensor::SensorId::DENSE_MATCHER0, "threshold", config.threshold);
      drv_.setSensorConfigParam(visensor::SensorId::DENSE_MATCHER0, "lr_check", config.lr_check);
    }
    // ========================= TRIGGER  ==========================
    if (drv_.isSensorPresent(visensor::SensorId::EXTERNAL_TRIGGER0)) {
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "enable_trigger_0", config.enable_trigger_0);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "enable_trigger_1", config.enable_trigger_1);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "enable_trigger_2", config.enable_trigger_2);

      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_0_direction", config.trigger_0_direction);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_1_direction", config.trigger_1_direction);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_2_direction", config.trigger_2_direction);

      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_0_rate", 100000.0/config.trigger_0_rate);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_1_rate", 100000.0/config.trigger_1_rate);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_2_rate", 100000.0/config.trigger_2_rate);

      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_0_offset", config.trigger_0_offset);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_1_offset", config.trigger_1_offset);
      drv_.setSensorConfigParam(visensor::SensorId::EXTERNAL_TRIGGER0, "trigger_2_offset", config.trigger_2_offset);
    }
  }

}

bool ViSensor::calibrationServiceCallback(visensor_msgs::visensor_calibration_service::Request& req,
                                          visensor_msgs::visensor_calibration_service::Response& res) {
  for (auto i : camera_imu_calibrations_)
    res.calibration.push_back(i.second);
  return true;
}

bool ViSensor::getRosCameraConfig(const SensorId::SensorId& camera_id, sensor_msgs::CameraInfo& cam_info)
{
  ViCameraCalibration camera_calibration;
  try {
    ViCameraCalibration tmp;
    std::vector<ViCameraCalibration> calibrations;
    drv_.getSelectedCameraCalibration(&tmp, camera_id);
    if (tmp.lens_model_->type_ != ViCameraLensModel::LensModelTypes::RADTAN ||
        tmp.projection_model_->type_ != ViCameraProjectionModel::ProjectionModelTypes::PINHOLE) {
      ROS_WARN("No radtan and pinhole calibration specified. Therefore the factory calibration is choosen for the ROS sensor message.\n");
      calibrations = drv_.getCameraCalibrations(camera_id, 0, tmp.is_flipped_,
                                                ViCameraLensModel::LensModelTypes::RADTAN, ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
      if (calibrations.size() == 0) {
        ROS_ERROR("No corresponding Factory Calibration found\n");
        return false;
      }
      camera_calibration = calibrations.front();
    }
    else {
      camera_calibration = tmp;
    }
  } catch (visensor::exceptions const & ex) {
    ROS_ERROR("getRosCameraConfig: catch following Exception: %s\n", ex.what());
    return false;
  }

  int image_width = camera_calibration.resolution_[0];
  int image_height = camera_calibration.resolution_[1];

  double c[9];
  double d[5];

  std::vector<double> lens_coefficients = camera_calibration.lens_model_->getCoefficients();
  std::fill(d, d + 5, 0);
  for ( unsigned int i = 0; i < lens_coefficients.size(); ++i ) {
    d[i] = lens_coefficients.at(i);
  }

  ViCameraProjectionModelPinhole::Ptr cam_projection_model =
      camera_calibration.getProjectionModel<ViCameraProjectionModelPinhole>();
  c[0] = cam_projection_model->focal_length_u_;
  c[1] = 0.0;
  c[2] = cam_projection_model->principal_point_u_;
  c[3] = 0.0;
  c[4] = cam_projection_model->focal_length_v_;
  c[5] = cam_projection_model->principal_point_v_;
  c[6] = 0.0;
  c[7] = 0.0;
  c[8] = 1.0;

  if (cam_info.D.size() != 5)
    cam_info.D.resize(5);

  for (int i = 0; i < 5; i++) {
    cam_info.D[i] = d[i];
  }

  for (int i = 0; i < 9; i++) {
    cam_info.K[i] = c[i];
  }

  cam_info.R.assign(0.0);
  cam_info.R[0] = cam_info.R[4] = cam_info.R[8] = 1.0;

  for (int i = 0; i < 3; i++) {
    cam_info.P[i] = c[i];
  }
  cam_info.P[3] = 0.0;
  for (int i = 4; i < 7; i++) {
    cam_info.P[i] = c[i-1];
  }
  cam_info.P[7] = 0.0;
  for (int i = 8; i < 11; i++) {
    cam_info.P[i] = c[i-2];
  }
  cam_info.P[11] = 0.0;

  cam_info.width = image_width;
  cam_info.height = image_height;

  cam_info.binning_x = 1;
  cam_info.binning_y = 1;

  cam_info.distortion_model = std::string("plumb_bob");

  return true;
}

bool ViSensor::getRosStereoCameraConfig(const SensorId::SensorId& camera_id_0,
                                          sensor_msgs::CameraInfo& cam_info_0,
                                          const SensorId::SensorId& camera_id_1,
                                          sensor_msgs::CameraInfo& cam_info_1) {
  ViCameraCalibration camera_calibration_0, camera_calibration_1;
  try {
    ViCameraCalibration tmp;
    std::vector<ViCameraCalibration> calibrations;
    drv_.getSelectedCameraCalibration(&tmp, camera_id_0);
    if (tmp.lens_model_->type_ != ViCameraLensModel::LensModelTypes::RADTAN ||
        tmp.projection_model_->type_ != ViCameraProjectionModel::ProjectionModelTypes::PINHOLE) {
      ROS_WARN("No radtan and pinhole calibration specified. Therefore the factory calibration is choosen for the ROS sensor message.\n");
      calibrations = drv_.getCameraCalibrations(camera_id_0, 0, tmp.is_flipped_,
                                                ViCameraLensModel::LensModelTypes::RADTAN, ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
      if (calibrations.size() == 0) {
        ROS_ERROR("No corresponding Factory Calibration for cam left found\n");
        return false;
      }
      camera_calibration_0 = calibrations.front();
    }
    else {
      camera_calibration_0 = tmp;
    }
    drv_.getSelectedCameraCalibration(&tmp, camera_id_1);
    if (tmp.lens_model_->type_ != ViCameraLensModel::LensModelTypes::RADTAN ||
        tmp.projection_model_->type_ != ViCameraProjectionModel::ProjectionModelTypes::PINHOLE) {
      ROS_WARN("No radtan and pinhole calibration specified. Therefore the factory calibration is choosen for the ROS sensor message.\n");
      calibrations = drv_.getCameraCalibrations(camera_id_1, 0, tmp.is_flipped_,
                                                ViCameraLensModel::LensModelTypes::RADTAN, ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
      if (calibrations.size() == 0) {
        ROS_ERROR("No corresponding Factory Calibration for cam right found\n");
        return false;
      }
      camera_calibration_1 = calibrations.front();
    }
    else {
      camera_calibration_1 = tmp;
    }
  }
  catch (visensor::exceptions const &ex) {
    ROS_WARN("Could not read the camera configuration of cameras %i and %i. Exception: %s",
             camera_id_0, camera_id_1, ex.what());
    return false;
  }

  int image_width = camera_calibration_0.resolution_[0];
  int image_height = camera_calibration_0.resolution_[1];;

  double c0[9];
  double d0[5];
  double r0[9];
  double p0[12];
  double rot0[9];
  double t0[3];

  double c1[9];
  double d1[5];
  double r1[9];
  double p1[12];
  double rot1[9];
  double t1[3];

  double r[9];
  double t[3];

  std::vector<double> lens_coefficients = camera_calibration_0.lens_model_->getCoefficients();
  std::fill(d0, d0 + 5, 0);
  for ( unsigned int i = 0; i < lens_coefficients.size(); ++i ) {
    d0[i] = lens_coefficients.at(i);
  }
  ViCameraProjectionModelPinhole::Ptr cam0_projection_model =
      camera_calibration_0.getProjectionModel<ViCameraProjectionModelPinhole>();
  c0[0] = cam0_projection_model->focal_length_u_;
  c0[1] = 0.0;
  c0[2] = cam0_projection_model->principal_point_u_;
  c0[3] = 0.0;
  c0[4] = cam0_projection_model->focal_length_v_;
  c0[5] = cam0_projection_model->principal_point_v_;
  c0[6] = 0.0;
  c0[7] = 0.0;
  c0[8] = 1.0;

  lens_coefficients = camera_calibration_1.lens_model_->getCoefficients();
  std::fill(d1, d1 + 5, 0);
  for ( unsigned int i = 0; i < lens_coefficients.size(); ++i ) {
    d1[i] = lens_coefficients.at(i);
  }
  ViCameraProjectionModelPinhole::Ptr cam1_projection_model = camera_calibration_1.getProjectionModel<ViCameraProjectionModelPinhole>();
  c1[0] = cam1_projection_model->focal_length_u_;
  c1[1] = 0.0;
  c1[2] = cam1_projection_model->principal_point_u_;
  c1[3] = 0.0;
  c1[4] = cam1_projection_model->focal_length_v_;
  c1[5] = cam1_projection_model->principal_point_v_;
  c1[6] = 0.0;
  c1[7] = 0.0;
  c1[8] = 1.0;

  for (int i = 0; i < 9; ++i) {
    rot0[i] = camera_calibration_0.R_[i];
    rot1[i] = camera_calibration_1.R_[i];
  }
  for (int i = 0; i < 3; ++i) {
    t0[i] = camera_calibration_0.t_[i];
    t1[i] = camera_calibration_1.t_[i];
  }

  Eigen::Map < Eigen::Matrix3d > RR0(rot0);
  Eigen::Map < Eigen::Vector3d > tt0(t0);
  Eigen::Map < Eigen::Matrix3d > RR1(rot1);
  Eigen::Map < Eigen::Vector3d > tt1(t1);

  Eigen::Matrix4d T0 = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d T1 = Eigen::Matrix4d::Zero();

  T0.block<3, 3>(0, 0) = RR0;
  T0.block<3, 1>(0, 3) = tt0;
  T0(3, 3) = 1.0;
  T1.block<3, 3>(0, 0) = RR1;
  T1.block<3, 1>(0, 3) = tt1;
  T1(3, 3) = 1.0;

  Eigen::Matrix4d T_rel = Eigen::Matrix4d::Zero();
  T_rel = T1 * T0.inverse();

  Eigen::Map < Eigen::Matrix3d > R_rel(r);
  Eigen::Map < Eigen::Vector3d > t_rel(t);

  R_rel = T_rel.block<3, 3>(0, 0);
  t_rel << T_rel(0, 3), T_rel(1, 3), T_rel(2, 3);

  double r_temp[9];
  r_temp[0] = R_rel(0, 0);
  r_temp[1] = R_rel(0, 1);
  r_temp[2] = R_rel(0, 2);
  r_temp[3] = R_rel(1, 0);
  r_temp[4] = R_rel(1, 1);
  r_temp[5] = R_rel(1, 2);
  r_temp[6] = R_rel(2, 0);
  r_temp[7] = R_rel(2, 1);
  r_temp[8] = R_rel(2, 2);

  //cv::Mat wrapped(rows, cols, CV_32FC1, external_mem, CV_AUTOSTEP);
  cv::Mat C0(3, 3, CV_64FC1, c0, 3 * sizeof(double));
  cv::Mat D0(5, 1, CV_64FC1, d0, 1 * sizeof(double));
  cv::Mat R0(3, 3, CV_64FC1, r0, 3 * sizeof(double));
  cv::Mat P0(3, 4, CV_64FC1, p0, 4 * sizeof(double));

  cv::Mat C1(3, 3, CV_64FC1, c1, 3 * sizeof(double));
  cv::Mat D1(5, 1, CV_64FC1, d1, 1 * sizeof(double));
  cv::Mat R1(3, 3, CV_64FC1, r1, 3 * sizeof(double));
  cv::Mat P1(3, 4, CV_64FC1, p1, 4 * sizeof(double));

  cv::Mat R(3, 3, CV_64FC1, r_temp, 3 * sizeof(double));

  cv::Mat T(3, 1, CV_64FC1, t, 1 * sizeof(double));

  cv::Size img_size(image_width, image_height);

  cv::Rect roi1, roi2;
  cv::Mat Q;

  cv::stereoRectify(C0, D0, C1, D1, img_size, R, T, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY, 0,
                    img_size, &roi1, &roi2);

  if (cam_info_0.D.size() != 5)
    cam_info_0.D.resize(5);

  if (cam_info_1.D.size() != 5)
    cam_info_1.D.resize(5);

  for (int i = 0; i < 5; i++) {
    cam_info_0.D[i] = d0[i];
    cam_info_1.D[i] = d1[i];
  }
  for (int i = 0; i < 9; i++) {
    cam_info_0.K[i] = c0[i];
    cam_info_0.R[i] = R0.at<double>(i);
    cam_info_1.K[i] = c1[i];
    cam_info_1.R[i] = R1.at<double>(i);
  }
  for (int i = 0; i < 12; i++) {
    cam_info_0.P[i] = P0.at<double>(i);
    cam_info_1.P[i] = P1.at<double>(i);
  }
  cam_info_0.width = camera_calibration_0.resolution_[0];
  cam_info_1.width = camera_calibration_1.resolution_[0];

  cam_info_0.height = camera_calibration_0.resolution_[1];
  cam_info_1.height = camera_calibration_1.resolution_[1];

  cam_info_0.binning_x = 1;
  cam_info_0.binning_y = 1;
  cam_info_1.binning_x = 1;
  cam_info_1.binning_y = 1;

  cam_info_0.distortion_model = std::string("plumb_bob");
  cam_info_1.distortion_model = std::string("plumb_bob");
  return true;
}

bool ViSensor::precacheViCalibration(const SensorId::SensorId& camera_id) {

  ViCameraCalibration camera_calibration;
  visensor_msgs::visensor_calibration calibration;

  geometry_msgs::Pose T_CI;
  try {
    drv_.getSelectedCameraCalibration(&camera_calibration, camera_id);
  }
  catch (visensor::exceptions const &ex) {
    camera_imu_calibrations_.insert(
        std::pair<std::string, visensor_msgs::visensor_calibration>(ROS_CAMERA_NAMES.at(camera_id),
                                                                    calibration));
    ROS_WARN("Failed to generate ViCalibration message for camera %i. Exception: %s\n", camera_id,
             ex.what());
    return false;
  }
  tf::Matrix3x3 R_CI(camera_calibration.R_[0], camera_calibration.R_[3], camera_calibration.R_[6],
                     camera_calibration.R_[1], camera_calibration.R_[4], camera_calibration.R_[7],
                     camera_calibration.R_[2], camera_calibration.R_[5], camera_calibration.R_[8]);

  tf::Quaternion q_CI;
  R_CI.getRotation(q_CI);

  T_CI.orientation.x = q_CI.x();
  T_CI.orientation.y = q_CI.y();
  T_CI.orientation.z = q_CI.z();
  T_CI.orientation.w = q_CI.w();

  T_CI.position.x = camera_calibration.t_[0];
  T_CI.position.y = camera_calibration.t_[1];
  T_CI.position.z = camera_calibration.t_[2];

  calibration.T_CI = T_CI;

  calibration.dist_coeff = camera_calibration.lens_model_->getCoefficients();
  switch (camera_calibration.lens_model_->type_) {
    case ViCameraLensModel::LensModelTypes::RADTAN:
      calibration.dist_model = std::string("plumb_bob");
      break;
    case ViCameraLensModel::LensModelTypes::EQUIDISTANT:
      calibration.dist_model = std::string("equi");
      break;
    default:
      ROS_WARN("current lens model not supported");
      return false;
  }

  calibration.projection_coeff = camera_calibration.projection_model_->getCoefficients();
  switch (camera_calibration.projection_model_->type_) {
    case ViCameraProjectionModel::ProjectionModelTypes::PINHOLE: {
      ViCameraProjectionModelPinhole::Ptr cam_projection_model =
          camera_calibration.getProjectionModel<ViCameraProjectionModelPinhole>();
      calibration.projection_model = std::string("pinhole");
      calibration.principal_point.push_back(cam_projection_model->principal_point_u_); //principal_point[0]);
      calibration.principal_point.push_back(cam_projection_model->principal_point_v_); //principal_point[1]);

      calibration.focal_length.push_back(cam_projection_model->focal_length_u_);    //focal_point[0]);
      calibration.focal_length.push_back(cam_projection_model->focal_length_v_);    //focal_point[1]);
      break;
    }
    case ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL: {
      ViCameraProjectionModelOmnidirectional::Ptr cam_projection_model =
          camera_calibration.getProjectionModel<ViCameraProjectionModelOmnidirectional>();
      calibration.projection_model = std::string("omni");
      calibration.principal_point.push_back(cam_projection_model->principal_point_u_); //principal_point[0]);
      calibration.principal_point.push_back(cam_projection_model->principal_point_v_); //principal_point[1]);

      calibration.focal_length.push_back(cam_projection_model->focal_length_u_);    //focal_point[0]);
      calibration.focal_length.push_back(cam_projection_model->focal_length_v_);    //focal_point[1]);
      break;
    }
    default:
      ROS_WARN("current projection model not supported");
      return false;
  }

  calibration.image_width = camera_calibration.resolution_[0];
  calibration.image_height = camera_calibration.resolution_[1];

  calibration.cam_name = ROS_CAMERA_NAMES.at(camera_id);

  camera_imu_calibrations_.insert(std::pair<std::string, visensor_msgs::visensor_calibration>(ROS_CAMERA_NAMES.at(camera_id), calibration));

  camera_imu_transformations_.insert(std::pair < std::string, tf::Transform > (ROS_CAMERA_NAMES.at(camera_id),
          tf::Transform(q_CI, tf::Vector3(T_CI.position.x, T_CI.position.y, T_CI.position.z)).inverse()));

  return true;
}

}  //namespace visensor
