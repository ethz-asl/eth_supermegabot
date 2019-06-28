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

#ifndef VISENSOR_H_
#define VISENSOR_H_

#include <string>
#include <map>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/fill_image.h>
#ifdef EXPERT_MODE
#include <visensor_node/DriverConfig.h>
#else
#include <visensor_node/DriverMinimalConfig.h>
#endif
#include "visensor_msgs/visensor_imu.h"
#include "visensor_msgs/visensor_time_host.h"
#include "visensor_msgs/visensor_trigger.h"
#include "visensor_msgs/visensor_calibration_service.h"
#include "visensor_msgs/visensor_calibration.h"

#include <visensor/visensor.hpp>

#ifndef EXPERT_MODE
namespace visensor_node {
typedef DriverMinimalConfig DriverConfig;
} // visensor_node namespace
#endif

namespace visensor {
static const std::string CAMERA_FRAME_NAME = "camera";
static const std::string ROS_TOPIC_NAME = "visensor/";
static const int NUMBER_OF_SUPPORTED_CAMERAS = 4;

#ifdef EXPERT_MODE
static std::map<SensorId::SensorId, std::string> ROS_CAMERA_NAMES {
  { SensorId::CAM0, "cam0" },
  { SensorId::CAM1, "cam1" },
  { SensorId::CAM2, "cam2" },
  { SensorId::CAM3, "cam3" },
  { SensorId::CAM4, "cam4" },
  { SensorId::CAM5, "cam5" },
  { SensorId::CAM6, "cam6" },
  { SensorId::CAM7, "cam7" },
  { SensorId::FLIR0, "tau0" },
  { SensorId::FLIR1, "tau1" },
  { SensorId::FLIR2, "tau2" },
  { SensorId::FLIR3, "tau3" } };
static const std::map<SensorId::SensorId, std::string> ROS_CAMERA_FRAME_NAMES {
  { SensorId::CAM0, "cam0" },
  { SensorId::CAM1, "cam1" },
  { SensorId::CAM2, "cam2" },
  { SensorId::CAM3, "cam3" },
  { SensorId::CAM4, "cam4" },
  { SensorId::CAM5, "cam5" },
  { SensorId::CAM6, "cam6" },
  { SensorId::CAM7, "cam7" },
  { SensorId::FLIR0, "tau0" },
  { SensorId::FLIR1, "tau1" },
  { SensorId::FLIR2, "tau2" },
  { SensorId::FLIR3, "tau3" } };
#else
static std::map<SensorId::SensorId, std::string> ROS_CAMERA_NAMES {
  { SensorId::CAM0, "left" },
  { SensorId::CAM1, "right" },
  { SensorId::CAM2, "cam2" },
  { SensorId::CAM3, "cam3" },
  { SensorId::CAM4, "cam4" },
  { SensorId::CAM5, "cam5" },
  { SensorId::CAM6, "cam6" },
  { SensorId::CAM7, "cam7" },
  { SensorId::FLIR0, "tau0" },
  { SensorId::FLIR1, "tau1" },
  { SensorId::FLIR2, "tau2" },
  { SensorId::FLIR3, "tau3" } };
static const std::map<SensorId::SensorId, std::string> ROS_CAMERA_FRAME_NAMES {
// TODO: Equal camera frames only make sense for stereo camera pairs. Figure out
// how to handle this with the left/right frame names.
  { SensorId::CAM0, "left" },
  { SensorId::CAM1, "left" },
  { SensorId::CAM2, "left" },
  { SensorId::CAM3, "left" },
  { SensorId::CAM4, "left" },
  { SensorId::CAM5, "left" },
  { SensorId::CAM6, "left" },
  { SensorId::CAM7, "left" },
  { SensorId::FLIR0, "tau0" },
  { SensorId::FLIR1, "tau1" },
  { SensorId::FLIR2, "tau2" },
  { SensorId::FLIR3, "tau3" } };
#endif
static const std::map<SensorId::SensorId, std::string> ROS_IMU_NAMES {
  { SensorId::IMU0, "imu0" },
  { SensorId::IMU_CAM0, "mpu0" },
  { SensorId::IMU_CAM1, "mpu1" },
  { SensorId::IMU_CAM2, "mpu2" },
  { SensorId::IMU_CAM3, "tau0" } };
static const std::map<SensorId::SensorId, std::string> ROS_IMU_FRAME_NAMES {
  { SensorId::IMU0, "imu0" },
  { SensorId::IMU_CAM0, "mpu0" },
  { SensorId::IMU_CAM1, "mpu1" },
  { SensorId::IMU_CAM2, "mpu2" },
  { SensorId::IMU_CAM3, "tau0" } };

class ViSensor {
 public:
  ViSensor(ros::NodeHandle& nh, std::string sensor_ip,
           const std::map<SensorId::SensorId, int>& slot_ids,
           const std::map<SensorId::SensorId, int>& is_flipped,
           const std::map<SensorId::SensorId, visensor::ViCameraLensModel::LensModelTypes>& lens_types,
           const std::map<SensorId::SensorId, visensor::ViCameraProjectionModel::ProjectionModelTypes>& projection_types,
           const SensorId::SensorId& stereo_left_cam, const SensorId::SensorId& stereo_right_cam,
           bool stereo_flip_disable, bool use_time_sync);
  // Delegating constructor with cam0 cam1 stereo pair as defaults.
  ViSensor(ros::NodeHandle& nh, std::string sensor_ip,
           const std::map<SensorId::SensorId, int>& slot_ids,
           const std::map<SensorId::SensorId, int>& is_flipped,
           const std::map<SensorId::SensorId, visensor::ViCameraLensModel::LensModelTypes>& lens_types,
           const std::map<SensorId::SensorId, visensor::ViCameraProjectionModel::ProjectionModelTypes>& projection_types,
           bool use_time_sync);

  ~ViSensor();
  void startSensors( std::map<SensorId::SensorId, int>& cam_rate, const int com_rate_global,
                    const unsigned int imu_rate, const unsigned int trigger_rate);

  //sensor callbacks
  void imuCallback(boost::shared_ptr<ViImuMsg> imu_ptr, ViErrorCode error);
  void frameCallback(ViFrame::Ptr frame_ptr, ViErrorCode error);
  void denseCallback(ViFrame::Ptr frame_ptr, ViErrorCode error);
  void frameCornerCallback(ViFrame::Ptr frame_ptr, ViCorner::Ptr corners_ptr);
  void triggerCallback(ViExternalTriggerMsg::Ptr trigger_ptr);

  bool calibrationServiceCallback(visensor_msgs::visensor_calibration_service::Request &req,
                                  visensor_msgs::visensor_calibration_service::Response &res);
  //dynamic reconfigure callback
  void configCallback(visensor_node::DriverConfig &config, uint32_t level);

 private:
  void init(const std::string& sensor_ip, const std::map<SensorId::SensorId, int>& slot_ids,
            const std::map<SensorId::SensorId, int>& is_flipped,
            const std::map<SensorId::SensorId, visensor::ViCameraLensModel::LensModelTypes>& lens_types,
            const std::map<SensorId::SensorId, visensor::ViCameraProjectionModel::ProjectionModelTypes>& projection_types,
            bool stereo_flip_disable);
  bool getRosCameraConfig(const SensorId::SensorId& camera_id, sensor_msgs::CameraInfo& cam_info);
  bool getRosStereoCameraConfig(const SensorId::SensorId& camera_id_0, sensor_msgs::CameraInfo& cam_info_0,
                                const SensorId::SensorId& camera_id_1, sensor_msgs::CameraInfo& cam_info_1);
  bool precacheViCalibration(const SensorId::SensorId& camera_id);

 private:
  ros::NodeHandle nh_;

  std::map<SensorId::SensorId, ros::NodeHandle> nhc_;
  std::map<SensorId::SensorId, image_transport::ImageTransport> itc_;
  std::map<SensorId::SensorId, image_transport::CameraPublisher> image_pub_;
  std::map<SensorId::SensorId, sensor_msgs::CameraInfo> cinfo_;

  std::map<SensorId::SensorId, ros::Publisher> imu_pub_;
  std::map<SensorId::SensorId, ros::Publisher> imu_custom_pub_;
  std::map<SensorId::SensorId, ros::Publisher> calibration_pub_;

  ros::Publisher pub_time_host_;
  ros::Publisher trigger_pub_;
  ros::ServiceServer calibration_service_;

  ViSensorDriver drv_;

  std::vector<SensorId::SensorId> list_of_available_sensors_;
  std::vector<SensorId::SensorId> list_of_camera_ids_;
  std::vector<SensorId::SensorId> list_of_dense_ids_;
  std::vector<SensorId::SensorId> list_of_imu_ids_;
  std::vector<SensorId::SensorId> list_of_trigger_ids_;
  SensorId::SensorId stereo_left_cam_;
  SensorId::SensorId stereo_right_cam_;


  std::map<std::string, visensor_msgs::visensor_calibration> camera_imu_calibrations_;
  std::map<std::string, tf::Transform> camera_imu_transformations_;

  tf::TransformBroadcaster br_;

  dynamic_reconfigure::Server<visensor_node::DriverConfig> dr_srv_;

  visensor_node::DriverConfig config_;

  bool use_time_sync_;
};

}  //namespace visensor

#endif /* VISENSOR_H_ */
