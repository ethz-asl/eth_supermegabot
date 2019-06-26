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

#ifndef VISENSOR_API_HPP_
#define VISENSOR_API_HPP_

#include <stdint.h>
#include <string>
#include <vector>

#include <boost/function.hpp>

#include "visensor/visensor_config.hpp"
#include "visensor/visensor_constants.hpp"
#include "visensor/visensor_datatypes.hpp"

namespace visensor {

/**
 * \class ViSensorDriver
 *
 * \brief This class contains the driver functionality to the physical Vi-Sensor.
 *
 * The Vi-Sensor is connected over ETH to the host. The sensor can be found with net with a DHCP server or without.
 * An object of the driver is only able to connect to at most one Vi-Sensor.
 *
 * \author Michael Burri, Janosch Nikolic, Joern Rehder, Stefan Leutenegger, Thomas Schneider, Pascal Gohl, Sammy Omari
 *
 * \date 2015/05/12
 *
 * contact: pascal.gohl at skybotix.com
 */
class DSO_EXPORT ViSensorDriver {
 public:
  typedef boost::shared_ptr<ViSensorDriver> Ptr;

  class Impl;

  /**
   * Constructor for the VI-Sensor driver object.
   * Set up a IP stack on the host.
   *
   */
  ViSensorDriver();

  /**
   * Destructor for the VI-Sensor driver object.
   *
   */
  ~ViSensorDriver();

  /**
   * Search VI-Sensors in network with a broadcast message specific contain and return a list of found devices.
   *
   * @param hostname_list list of detected sensors
   */
  void getAutoDiscoveryDeviceList(ViDeviceList &hostname_list);

  /** Init using autodiscovery (select first sensor that responded)
   *
   * @return IP address of the VI-Sensor to which the connection was made
   */
  std::string init();

  /** connect to specific VI-Sensor by specifying its host/ip address
   *
   * @param host_ip host/ip address
   */
  void init(std::string host_ip);

  /** Start data acquisition of a certain sensor available on the VI-Sensor
   *
   * @param sensor_id The unique ID of the sensor
   * @param rate The frequency in Hz the sensor should be sampled
   */
  void startSensor(visensor::SensorId::SensorId sensor_id, uint32_t rate =
                       CAMERA_FREQUENCY);

  /** Stop the data of a sensor
   *
   * @param sensor_id The ID of the sensor which should be stopped
   */
  void stopSensor(visensor::SensorId::SensorId sensor_id);

  /** Setting a sensor configuration parameter
   *
   * @param sensor_id The ID of the sensor which should be configured
   * @param cmd       The name of the configuration parameter
   * @param value     The configuration value
   */
  void setSensorConfigParam(visensor::SensorId::SensorId sensor_id, const std::string cmd,
                            int value);

  /** Check if a sensor is connected to the VI-Sensor
   *
   * @param sensor_id The ID of the sensor which should be checked
   */
  bool isSensorPresent(const SensorId::SensorId sensor_id) const;

  /** Start all available cameras
   *
   */
  void startAllCameras(uint32_t rate = CAMERA_FREQUENCY);

  /** register the callback function, which will be called for every new frame
   *
   * @param callback The function point, which will be called for new data
   */
  void setCameraCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback);

  /** start all corner detectors
   * @brief they will run with the frequency of the cameras
   */
  void startAllCorners();

  /** register the callback function, which will be called for every new frame
   *
   * @param callback The function point, which will be called for new data
   */
  void setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback);

  /** start all available IMUs
   *
   * @param rate The frequency in Hz at which the IMU should be sampled
   */
  void startAllImus(uint32_t rate = IMU_FREQUENCY);

  /** register the callback function, which will be called for every new IMU measurement package
   *
   * @param callback The function point, which will be called for new data
   */
  void setImuCallback(boost::function<void(ViImuMsg::Ptr, ViErrorCode)> callback);

  /** start all available trigger lines
   *
   * @param rate The frequency in Hz at which trigger will send impulses (ignored when configured as input)
   */
  void startAllExternalTriggers(uint32_t rate);

  /** register the callback function, which will be called for every new trigger event
   *
   * @param callback The function point, which will be called for new data
   */
  void setExternalTriggerCallback(boost::function<void(ViExternalTriggerMsg::Ptr)> callback);
  
  /** Configure the external triggers 
   *
   * @param config Configuration struct
   */
  void setExternalTriggerConfig(const ViExternalTriggerConfig config);

  /** start all available dense matchers
   *
   */
  void startAllDenseMatchers();

  /** register the callback function, which will be called for every new dense image
   *
   * @param callback The function point, which will be called for new data
   */
  void setDenseMatcherCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback);

  /** register the callback function, which will be called for is called with
   * synchronized images and corresponding corners
   *
   * @param callback The function point, which will be called for new data
   */
  void setFramesCornersCallback(boost::function<void(ViFrame::Ptr, ViCorner::Ptr)> callback);

  std::vector<SensorId::SensorId> getListOfSensorIDs() const;
  std::vector<SensorId::SensorId> getListOfCameraIDs() const;
  std::vector<SensorId::SensorId> getListOfDenseIDs() const;
  std::vector<SensorId::SensorId> getListOfCornerIDs() const;
  std::vector<SensorId::SensorId> getListOfImuIDs() const;
  std::vector<SensorId::SensorId> getListOfTriggerIDs() const;
  uint32_t getFpgaId() const;

  /**
   * Saves the VISensor ID
   *
   * In case of an Error an Exception is thrown.
   */
  void setViSensorId(const int vi_sensor_id);

  /**
   * Returns the saved VISensor ID.
   *
   * @return VISensor id if successful. In case of an Error an Exception is thrown.
   */
  int getViSensorId() const;

  /**
   * Gets user configuration parameters which were saved previous on the visensor
   * in case of an error an visensor exception (ConfigException) is thrown.
   *
   * @param key:    identifier for the parameter
   * @param value:  return value
   */
  void getUserConfiguration(const std::string& key, int * value) const;
  void getUserConfiguration(const std::string& key, std::string* value) const;

  /**
   * sets an user configuration parameter
   *
   * in case of an error an visensor exception (ConfigException) is thrown.
   *
   * @param key:    identifier for the parameter
   * @param value:  value to set
   */
  void setUserConfiguration(const std::string& key, const int& value);
  void setUserConfiguration(const std::string& key, const std::string& value);

  /**
   * returns the specific camera calibration.
   *
   * If more than one calibration configuration matches to the input values the first calibration is returned.
   * In case of an error, a exception is thrown.
   *
   * @param cam_id:     filters the calibrations for sensor id (<0 means don't care)
   * @param slot_id:    defines the used slot. 0 is for factory calibration, >0 for customer. <0 means don't care
   * @param is_flipped: filters calibration for flipped (1) or non flipped (0) calibrations. A flipped
   *                    camera is turned by 180° so that the image has to be flipped horizontally and vertically.
   *                    <0 means don't care
   * @param lens_model_type:  filters calibration for specific lens model type. set type to unknown if the lens model type does not care
   * @param projection_model_type:  filters calibration for specific projection model type. set type to unknown if the projection model  type does not care
   *
   * @return returns a copy of the calibration
   *
   */
  std::vector<ViCameraCalibration> getCameraCalibrations(SensorId::SensorId cam_id) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(SensorId::SensorId cam_id,
                                                         int slot_id) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(SensorId::SensorId cam_id,
                                                         ViCameraLensModel::LensModelTypes lens_model_type,
                                                         ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(SensorId::SensorId cam_id,
                                                         int slot_id,
                                                         ViCameraLensModel::LensModelTypes lens_model_type,
                                                         ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(SensorId::SensorId cam_id,
                                                         int slot_id,
                                                         int is_flipped,
                                                         ViCameraLensModel::LensModelTypes lens_model_type,
                                                         ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const;


  /**
   * adds the given calibration and saves it to the sensor
   *
   * Calibrations with slot_id_ 0 aren't able to set because this is the factory calibration
   * In case of an error, a exception is thrown.
   *
   * @param cali:     calibration to save.
   */
  void setCameraCalibration(const ViCameraCalibration& cali) const;

  /**
   * Deletes all the camera calibrations which fit to the filters
   *
   * If more than one calibration configuration matches to the input values all of them were deleted
   * In case of an error, a exception is thrown.
   *
   * @param cam_id:     filters the calibrations for sensor id (<0 means don't care)
   * @param slot_id:    defines the used slot. 0 is for factory calibration and not able to delete over this api, >0 for customer. <0 means don't care
   * @param is_flipped: filters calibration for flipped (1) or non flipped (0) calibrations. A flipped
   *                    camera is turned by 180° so that the image has to be  flipped horizontally and vertically.
   *                    <0 means don't care
   * @param lens_model_type:  filters calibration for specific lens model type. set type to unknown if the lens model type does not care
   * @param projection_model_type:  filters calibration for specific projection model type. set type to unknown if the projection model  type does not care
   *
   * @return true if at least one calibration config is erased
   */
  bool cleanCameraCalibrations(const SensorId::SensorId cam_id,
                               const int slot_id,
                               const int is_flipped,
                               const ViCameraLensModel::LensModelTypes lens_model_type,
                               const ViCameraProjectionModel::ProjectionModelTypes projection_model_type);
  bool cleanCameraCalibrations(const SensorId::SensorId cam_id);
  bool cleanCameraCalibrations();

  /**
   * configure/set which calibration should be used for the camera
   *
   * @param cam_id:     filters the calibrations for sensor id
   * @param slot_id:    defines the used slot. 0 is for factory calibration, >0 for customer. <0 means don't care
   * @param is_flipped: filters calibration for flipped (1) or non flipped (0) calibrations. A flipped
   *                    camera is turned by 180° so that the image has to be flipped horizontally and vertically.
   *                    <0 means don't care
   * @param lens_model_type:  filters calibration for specific lens model type. set type to unknown if the lens model type does not care
   * @param projection_model_type:  filters calibration for specific projection model type.
   *                    set type to unknown if the projection model  type does not care
   *
   * @return copy of a vector of matching calibrations
   */
  void selectCameraCalibration(const SensorId::SensorId cam_id,
                                 const int slot_id,
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

  bool isStereoCameraFlipped() const;

  //serial port bridge
  void sendSerialData(ViSerialData::Ptr data) const;
  void setSerialCallback(boost::function<void(ViSerialData::Ptr)> callback);
  void setSerialDelimiter(const char serial_id, const std::string delimiter) const;
  void setSerialBaudrate(const char serial_id, const unsigned int baudrate) const;

  /**
   * Download a file from a local path to the Vi-Sensor.
   *
   * This function is not implemented on the Vi-Sensor firmware!!!!!!
   *
   * @param local_path  path to the local file
   * @param remote_path path where the file should be copied. On the Vi-Sensor is also a Linux running. Be award that there are some non writable sections.
   *
   */
  void downloadFile(std::string local_path, std::string remote_path);

  /**
   * Upload a file from the Vi-Sensor to a local path.
   *
   * This function is not implemented on the Vi-Sensor firmware!!!!!!
   *
   * @param local_path  path to the local file
   * @param remote_path path where the file should be copied. On the Vi-Sensor is also a Linux running. Be award that there are some non writable sections.
   *
   */
  void uploadFile(std::string local_path, std::string remote_path);

 private:
  Impl* pImpl_;

 public:
  //private API access
  Impl* getPrivateApiAccess() { return pImpl_; }
};

}  //namespace visensor

#endif /* VISENSOR_API_HPP_ */
