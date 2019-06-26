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

#ifndef VISENSOR_IMPL_HPP_
#define VISENSOR_IMPL_HPP_

#include <config/config.hpp>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "config/visensor_configuration.hpp"
#include "visensor/visensor.hpp"

#include "networking/connection.hpp"
#include "networking/file_transfer.hpp"
#include "synchronization/frame_corner_synchronizer.hpp"

// sensors
#include "sensors/sensor_factory.hpp"
#include "sensors/sensor.hpp"
#include "sensors/camera_mt9v034.hpp"
#include "sensors/camera_tau640.hpp"
#include "sensors/imu_adis16448.hpp"
#include "sensors/imu_adis16488.hpp"
#include "sensors/corner_mt9v034.hpp"


namespace visensor {

class ViSensorDriver::Impl {
 public:
  Impl();
  virtual ~Impl();

  void getAutoDiscoveryDeviceList(ViDeviceList &hostname_list);

  void init(std::string hostname);
  std::string initAutodiscovery();

  void startSensor(SensorId::SensorId sensor_id, uint32_t rate = CAMERA_FREQUENCY);
  void stopSensor(SensorId::SensorId sensor_id);
  void setSensorConfigParam(SensorId::SensorId sensor_id, const std::string cmd, uint16_t value);
  bool isSensorPresent(const SensorId::SensorId sensor_id) const;

  void startAllCameras(uint32_t rate = CAMERA_FREQUENCY);
  void setCameraCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback);

  void startAllCorners();
  void setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback);

  void startAllImus(uint32_t rate = IMU_FREQUENCY);
  void setImuCallback(boost::function<void(ViImuMsg::Ptr, ViErrorCode)> callback);

  void startAllExternalTriggers(uint32_t rate);
  void setExternalTriggerCallback(
      boost::function<void(ViExternalTriggerMsg::Ptr)> callback);
  void setExternalTriggerConfig(const ViExternalTriggerConfig config);

  void startAllDenseMatchers();
  void setDenseMatcherCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback);

  std::vector<ViCameraCalibration> getCameraCalibrations(const SensorId::SensorId cam_id) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(const SensorId::SensorId cam_id,
                                                         const int slot_id) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(const SensorId::SensorId cam_id,
                                                         const ViCameraLensModel::LensModelTypes lens_model_type,
                                                         const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(const SensorId::SensorId cam_id,
                                                         const int slot_id,
                                                         const ViCameraLensModel::LensModelTypes lens_model_type,
                                                         const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(const SensorId::SensorId cam_id,
                                                         const int slot_id,
                                                         const int is_flipped,
                                                         const ViCameraLensModel::LensModelTypes lens_model_type,
                                                         const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const;

  void setCameraCalibration(const ViCameraCalibration& calib);

  bool cleanCameraCalibrations(const SensorId::SensorId cam_id,
                               const int slot_id,
                               const int is_flipped,
                               const ViCameraLensModel::LensModelTypes lens_model_type,
                               const ViCameraProjectionModel::ProjectionModelTypes projection_model_type);
  bool cleanCameraCalibrations(const SensorId::SensorId cam_id,
                               const int slot_id);
  bool cleanCameraCalibrations(const SensorId::SensorId cam_id);
  bool cleanCameraCalibrations();

  void setCameraFactoryCalibration(const ViCameraCalibration calib);

  void setViSensorId(const int vi_sensor_id);
  int getViSensorId() const;

  void getUserConfiguration(const std::string& key, int* value) const;
  void getUserConfiguration(const std::string& key, std::string* value) const;

  void setUserConfiguration(const std::string& key, const int& value);
  void setUserConfiguration(const std::string& key, const std::string& value);

  bool isStereoCameraFlipped();
  // is called with synchronized images and corresponding corners
  void setFramesCornersCallback(
      boost::function<
          void(ViFrame::Ptr, ViCorner::Ptr)> callback);

  std::vector<SensorId::SensorId> getListOfSensorIDs() const;
  std::vector<SensorId::SensorId> getListOfCameraIDs() const;
  std::vector<SensorId::SensorId> getListOfDenseIDs() const;
  std::vector<SensorId::SensorId> getListOfCornerIDs() const;
  std::vector<SensorId::SensorId> getListOfImuIDs() const;
  std::vector<SensorId::SensorId> getListOfTriggerIDs() const;
  uint32_t getFpgaId() const;

  /**
   * configure/set which calibration should be used for the camera
   *
   * @param cam_id:     filters the calibrations for sensor id
   * @param slot_id:    defines the used slot. 0 is for factory calibration, >0 for customer. <0 means don't care
   * @param is_flipped: filters calibration for flipped (1) or non flipped (0) calibrations. <0 means don't care
   * @param lens_model_type:  filters calibration for specific lens model type. set type to unknown if the lens model type does not care
   * @param projection_model_type:  filters calibration for specific projection model type. set type to unknown if the projection model  type does not care
   *
   * @return copy of a vector of matching calibrations
   */
  void selectCameraCalibration(const SensorId::SensorId cam_id,
                                 const int slot_id,
                                 const int is_flipped,
                                 const ViCameraLensModel::LensModelTypes lens_model_type,
                                 const ViCameraProjectionModel::ProjectionModelTypes projection_model_type);
  void selectCameraCalibration(const int slot_id,
                                 const int is_flipped,
                                 const ViCameraLensModel::LensModelTypes lens_model_type,
                                 const ViCameraProjectionModel::ProjectionModelTypes projection_model_type);

  void selectCameraCalibration();

  /**
   * returns the configured/ used calibration for the camera
   *
   * @param calibration returns the used calibration
   * @param cam_id:     sensor id
   *
   */
  void getSelectedCameraCalibration(ViCameraCalibration* usedCalibration, const SensorId::SensorId camera_id) const;

  void downloadFile(std::string& local_path, std::string& remote_path);
  void uploadFile(std::string& local_path, std::string& remote_path);

  void sendSerialData(ViSerialData::Ptr data);
  void setSerialCallback(boost::function<void(ViSerialData::Ptr)> callback);
  void setSerialDelimiter(const char serial_id, const std::string delimiter);
  void setSerialBaudrate(const char serial_id, const unsigned int baudrate);

 private:
  bool isInitialized();

  IpConnection::Ptr ip_connection_;
  SerialHost::Ptr serial_host_;

  Sensor::IdMap sensors_;
  boost::thread_group sensor_threads_;

  bool initialized_;

  ViSensorConfiguration::Ptr config_;

  std::vector<FrameCornerSynchronizer*> cam_corner_syncronizer_;
};
}  //namespace visensor

#endif
