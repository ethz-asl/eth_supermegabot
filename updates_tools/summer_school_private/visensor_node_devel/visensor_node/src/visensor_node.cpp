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
#include "visensor.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visensor_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  //default sensor parameters
  int cam_rate_global;
  int imu_rate;
  int trigger_rate;
  std::string sensor_ip;
  bool use_time_sync;
  std::map<visensor::SensorId::SensorId, int> cam_rates;
  std::map<visensor::SensorId::SensorId, int> slot_ids;
  std::map<visensor::SensorId::SensorId, int> is_flipped;
  std::map<visensor::SensorId::SensorId, visensor::ViCameraLensModel::LensModelTypes> lens_types;
  std::map<visensor::SensorId::SensorId, visensor::ViCameraProjectionModel::ProjectionModelTypes> projection_types;

  //Read values from ROS or set to default value
  private_nh.param("camRate", cam_rate_global, CAMERA_FREQUENCY);
  private_nh.param("imuRate", imu_rate, IMU_FREQUENCY);
  private_nh.param("triggerRate", trigger_rate, IMU_FREQUENCY);
  private_nh.param("sensorIp", sensor_ip, std::string("0.0.0.0"));
  private_nh.param("useTimeSync", use_time_sync, false);

  XmlRpc::XmlRpcValue my_list;
  if (nh.getParam("cameras", my_list)){
    // check for sanity - there must be at least one entry
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(my_list.size()>0);
    for (int32_t i = 0; i < my_list.size(); ++i) {
      // sanity check
      ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(my_list[i].hasMember("camera_number"));
      ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

      visensor::SensorId::SensorId camera_number =
          static_cast<visensor::SensorId::SensorId>((int)my_list[i]["camera_number"]);

      ROS_INFO_STREAM("[visensor_node] Getting config for camera " << camera_number << " from the yaml file\n" << std::endl);

      if (my_list[i].hasMember("rate"))
        cam_rates[camera_number] = (int) my_list[i]["rate"];
      else
        cam_rates[camera_number] = cam_rate_global;

      if (my_list[i].hasMember("slot_id"))
        slot_ids[camera_number] = (int) my_list[i]["slot_id"];
      else
        slot_ids[camera_number] = -1;

      if (my_list[i].hasMember("is_flipped"))
        is_flipped[camera_number] = (int) my_list[i]["is_flipped"];
      else
        is_flipped[camera_number] = -1;

      if (my_list[i].hasMember("lens_type")) {
        std::string lens_type;
        lens_type.assign(my_list[i]["lens_type"]);
        if (lens_type == std::string("equidistant")) {
          lens_types[camera_number] = visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT;
        } else if (lens_type == std::string("radtan")) {
          lens_types[camera_number] = visensor::ViCameraLensModel::LensModelTypes::RADTAN;
        } else {
          ROS_WARN("Lens Model is not supported");
          return 0;
        }
      } else
        lens_types[camera_number] = visensor::ViCameraLensModel::LensModelTypes::UNKNOWN;
      if (my_list[i].hasMember("projection_type")) {
        std::string projection_type;
        projection_type.assign(my_list[i]["projection_type"]);
        if (projection_type == std::string("pinhole")) {
          projection_types[camera_number] = visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE;
        } else if (projection_type == std::string("omnidirectional")) {
          projection_types[camera_number] = visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL;
        } else {
          ROS_WARN("Projection Model is not supported");
          return 0;
        }
      } else
        projection_types[camera_number] = visensor::ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN;
    }
  }
  else {
    ROS_WARN("[visensor_node] No config loaded from a yaml file\n");
  }

  // Get the names of the left and right stereo pair - if no parameters
  // specified, default to cam0 (left) and cam1 (right).
  visensor::SensorId::SensorId stereo_left_cam = visensor::SensorId::CAM0;
  visensor::SensorId::SensorId stereo_right_cam = visensor::SensorId::CAM1;

  // These settings only make sense in expert mode.
  bool stereo_flip_disable = false;
  #ifdef EXPERT_MODE
  std::string left_name, right_name;
  if (private_nh.getParam("stereo_left_cam", left_name) &&
      private_nh.getParam("stereo_right_cam", right_name)) {

    for (const std::pair<const visensor::SensorId::SensorId, std::string>& cam_name_pair : visensor::ROS_CAMERA_NAMES) {
      if (cam_name_pair.second == left_name) {
        stereo_left_cam = cam_name_pair.first;
      }
      if (cam_name_pair.second == right_name) {
        stereo_right_cam = cam_name_pair.first;
      }
    }
    stereo_flip_disable = true;
    // Will default to cam0, cam1 if the names are never found.
    ROS_INFO_STREAM("Setting left camera to " << left_name
                    << " and right camera to " << right_name);
  }
  #endif

  visensor::ViSensor vi_sensor(nh, sensor_ip, slot_ids, is_flipped, lens_types, projection_types,
                               stereo_left_cam, stereo_right_cam, stereo_flip_disable, use_time_sync);
  vi_sensor.startSensors(cam_rates, cam_rate_global, imu_rate, trigger_rate);

  ros::spin();

  return 0;
}
