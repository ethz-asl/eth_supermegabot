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

#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include "networking/auto_discovery.hpp"
#include "sensors/sensor_factory.hpp"
#include "sensors/dense_matcher.hpp"
#include "visensor/visensor_exceptions.hpp"

#include "visensor_impl.hpp"
#include "serial_bridge/serial_host.hpp"

namespace visensor {

ViSensorDriver::Impl::Impl()
: initialized_(false)
{
  ip_connection_ = boost::make_shared<IpConnection>();
}

std::string ViSensorDriver::Impl::initAutodiscovery()
{
  boost::asio::io_service io_service_;
  AutoDiscovery sensor_finder(13779);
  ViDeviceList sensor_ip_list = sensor_finder.findSensor();

  if (sensor_ip_list.empty())
    throw visensor::exceptions::ConnectionException("Autodiscovery: could not find a sensor.");

  if (sensor_ip_list.size() > 1)
    throw visensor::exceptions::ConnectionException("Autodiscovery: found more than one sensor. Please set sensor IP.");

  //found only one sensor, thus connect to it
  std::string sensor_ip = sensor_ip_list[0];

  //init with found IP address
  init(sensor_ip);

  return sensor_ip;
}

void ViSensorDriver::Impl::getAutoDiscoveryDeviceList(ViDeviceList &hostname_list)
{
  boost::asio::io_service io_service_;
  AutoDiscovery sensor_finder(13779);

  hostname_list = sensor_finder.findSensor();
}

void ViSensorDriver::Impl::init(std::string sensor_ip) {
  // create fpga interface
  ip_connection_->connect(sensor_ip);

  config_ = boost::make_shared<ViSensorConfiguration>(ip_connection_->file_transfer());

  if (!config_->isValid()) {
    config_->loadConfig();
  }

  VISENSOR_DEBUG("Connection established with VISensor %i\n", config_->getViSensorId());

  // get a list of all sensors connected to the fpga
  std::vector<IpComm::SensorInfo> sensor_list = ip_connection_->getAttachedSensors();

  // create all the sensor objects
  SensorFactory::createSensors(ip_connection_, &sensors_, &sensor_threads_);

  for (Sensor::IdMap::iterator it = sensors_.begin(); it != sensors_.end();
      ++it) {
    // inform comm about sensor
    ip_connection_->addSensor(it->second->id(), it->second);
    // init sensor
    it->second->init();
  }

  //initialize the serial bridge
  serial_host_ = boost::make_shared<SerialHost>();
  ip_connection_->registerSerialHost(serial_host_);


  //set initialized flag
  initialized_ = true;
}

ViSensorDriver::Impl::~Impl()
{
  try {
    sensor_threads_.interrupt_all();
    sensor_threads_.join_all();

    // stop all sensors
    for (Sensor::IdMap::iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      ip_connection_->stopSensor(it->second->id());
    }

  } catch (const std::exception &ex)
  {
    VISENSOR_DEBUG("ViSensorDriver::Impl exception in destructor: %s\n", ex.what());
  }
}

bool ViSensorDriver::Impl::isInitialized(void)
{
  return initialized_;
}

void ViSensorDriver::Impl::startAllCameras(uint32_t rate) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CAMERA_MT9V034
        || it->second->type() == SensorType::CAMERA_TAU640
        || it->second->type() == SensorType::CAMERA_TAU320)
      startSensor(it->first, rate);
  }
}

void ViSensorDriver::Impl::setCameraCallback(
    boost::function<void(ViFrame::Ptr, ViErrorCode)> callback) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CAMERA_MT9V034
        || it->second->type() == SensorType::CAMERA_TAU640
        || it->second->type() == SensorType::CAMERA_TAU320)
      it->second->setFrameCallback(callback);
  }
}

void ViSensorDriver::Impl::setImuCallback(
    boost::function<void(boost::shared_ptr<ViImuMsg>, ViErrorCode)> callback) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::IMU_ADIS16448
    	|| it->second->type() == SensorType::IMU_ADIS16488
        || it->second->type() == SensorType::MPU_9150)
      it->second->setUserCallback(callback);
  }
}

void ViSensorDriver::Impl::startSensor(SensorId::SensorId sensor_id, uint32_t rate) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  // check if id is valid
  if (sensors_.count(sensor_id) == 0)
    throw visensor::exceptions::SensorException("startSensor: Invalid sensor id: " + boost::lexical_cast<std::string>(sensor_id));

  //TODO(gohlp) move rate check to sensor class
  if (rate == 0 || rate > 1000)
    throw visensor::exceptions::SensorException("startSensor: Invalid rate for sensor: " + boost::lexical_cast<std::string>(sensor_id));

  // set sensor active
  sensors_.at(sensor_id)->startSensor(rate);

  // start triggering the sensor on the fpga
  ip_connection_->startSensor(sensors_.at(sensor_id)->id(), rate);
}

void ViSensorDriver::Impl::stopSensor(SensorId::SensorId sensor_id) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  // check if id is valid
  if (sensors_.count(sensor_id) == 0)
    throw visensor::exceptions::SensorException("stopSensor: Invalid sensor id: " + boost::lexical_cast<std::string>(sensor_id));

  // stop triggering the sensor on the fpga
  ip_connection_->stopSensor(sensors_.at(sensor_id)->id());

  sensors_.at(sensor_id)->stopSensor();
}

void ViSensorDriver::Impl::startAllImus(uint32_t rate) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::IMU_ADIS16448
          || it->second->type() == SensorType::IMU_ADIS16488           
          || it->second->type() == SensorType::MPU_9150)
          startSensor(it->first, rate);
  }
}

std::vector<SensorId::SensorId> ViSensorDriver::Impl::getListOfSensorIDs() const {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<SensorId::SensorId> list_of_sensors;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
    list_of_sensors.push_back(it->first);
  }
  return list_of_sensors;
}

std::vector<SensorId::SensorId> ViSensorDriver::Impl::getListOfCameraIDs() const {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<SensorId::SensorId> list_of_cameras;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::CAMERA_MT9V034
          || it->second->type() == SensorType::CAMERA_TAU640
          || it->second->type() == SensorType::CAMERA_TAU320)
    list_of_cameras.push_back(it->first);
  }
  return list_of_cameras;
}

std::vector<SensorId::SensorId> ViSensorDriver::Impl::getListOfDenseIDs() const {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<SensorId::SensorId> list_of_dense;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::DENSE_MATCHER)
    list_of_dense.push_back(it->first);
  }
  return list_of_dense;
}

std::vector<SensorId::SensorId> ViSensorDriver::Impl::getListOfImuIDs() const {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<SensorId::SensorId> list_of_imus;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::IMU_ADIS16448
          || it->second->type() == SensorType::IMU_ADIS16488
          || it->second->type() == SensorType::MPU_9150)
    list_of_imus.push_back(it->first);
  }
  return list_of_imus;
}

std::vector<SensorId::SensorId> ViSensorDriver::Impl::getListOfCornerIDs() const {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<SensorId::SensorId> list_of_corners;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::CORNER_MT9V034)
    list_of_corners.push_back(it->first);
  }
  return list_of_corners;
}

std::vector<SensorId::SensorId> ViSensorDriver::Impl::getListOfTriggerIDs() const {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<SensorId::SensorId> list_of_triggers;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::EXTERNAL_TRIGGER)
    list_of_triggers.push_back(it->first);
  }
  return list_of_triggers;
}

uint32_t ViSensorDriver::Impl::getFpgaId() const {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  return ip_connection_->getId();
}

void ViSensorDriver::Impl::setSensorConfigParam(SensorId::SensorId sensor_id,
                                                  std::string cmd,
                                                  uint16_t value) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  // check if id is valid
  if (sensors_.count(sensor_id) == 0)
    throw visensor::exceptions::SensorException("setSensorConfigParam: Invalid sensor id: " + boost::lexical_cast<std::string>(sensor_id));

  ViConfigMsg msg = sensors_.at(sensor_id)->getConfigParam(cmd, value);

  if (msg.valChanged) {
    VISENSOR_DEBUG(
        "configRequest \"%s\" : id %#X, addr %#X, reg %#X,val %#X type %#X \n",
        cmd.c_str(), msg.sensorId, msg.devAdress, msg.reg, msg.val,
        msg.comType);
    ip_connection_->writeConfig(msg.sensorId, msg.devAdress, msg.reg, msg.val,
                                msg.comType);
  }
}

void ViSensorDriver::Impl::downloadFile(std::string& local_path,
                                          std::string& remote_path) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  ip_connection_->file_transfer()->downloadFile(local_path, remote_path);
}

void ViSensorDriver::Impl::startAllExternalTriggers(uint32_t rate) {
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::EXTERNAL_TRIGGER)
      startSensor(it->first, rate);
  }
}

void ViSensorDriver::Impl::setExternalTriggerCallback(
    boost::function<void(ViExternalTriggerMsg::Ptr)> callback) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::EXTERNAL_TRIGGER)
        it->second->setUserCallback(callback);
  }
}

void ViSensorDriver::Impl::setExternalTriggerConfig(const ViExternalTriggerConfig config) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  //send configure packet
  //TODO(schneith): if there is more than one externalTrigger core, all will be
  //                configured with the same config... maybe extend with an id...
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::EXTERNAL_TRIGGER)
        ip_connection_->sendExternalTriggerConfig(it->second->id(), config);
  }
}

void ViSensorDriver::Impl::startAllDenseMatchers() {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::DENSE_MATCHER)
      startSensor(it->first, 1);
  }
}

void ViSensorDriver::Impl::setDenseMatcherCallback(
    boost::function<void(ViFrame::Ptr, ViErrorCode)> callback) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::DENSE_MATCHER)
      it->second->setFrameCallback(callback);
  }
}

void ViSensorDriver::Impl::selectCameraCalibration(const SensorId::SensorId cam_id,
                                                     const int slot_id,
                                                     const int is_flipped,
                                                     const ViCameraLensModel::LensModelTypes lens_model_type,
                                                     const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) {
  std::vector<ViCameraCalibration> calibrations;
  calibrations = getCameraCalibrations(cam_id, slot_id, is_flipped, lens_model_type, projection_model_type);

  if (calibrations.size() > 1)
    throw visensor::exceptions::ConfigException("More than one calibration found. Please specify the calibration to use more specific.");
  if (calibrations.size() == 0)
    throw visensor::exceptions::ConfigException("No calibration were found. Please select an existing calibration");

  VISENSOR_DEBUG("Camera %i is_flipped %i\n", cam_id, calibrations.front().is_flipped_);

  bool is_camera_flipped = calibrations.front().is_flipped_;
  if (cam_id !=visensor::SensorId::CAM0)//Taking care of legacy stuff...
    is_camera_flipped = !is_camera_flipped;

  setSensorConfigParam(cam_id, "row_flip", is_camera_flipped);
  setSensorConfigParam(cam_id, "column_flip", is_camera_flipped);
  config_->selectCameraCalibration(calibrations.front(), cam_id);
}

void ViSensorDriver::Impl::selectCameraCalibration(const int slot_id,
                                                     const int is_flipped,
                                                     const ViCameraLensModel::LensModelTypes lens_model_type,
                                                     const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) {
  //flip images if necessary
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CAMERA_MT9V034){
      selectCameraCalibration(static_cast<SensorId::SensorId>(it->first), slot_id, is_flipped, lens_model_type, projection_model_type);
    }
  }

  // apply calibrations
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::DENSE_MATCHER) {
      DenseMatcher::Ptr dense_matcher = boost::static_pointer_cast<DenseMatcher>(it->second);
      ViCameraCalibration cam0_calibration, cam1_calibration;
      config_->getSelectedCameraCalibration(&cam0_calibration, SensorId::SensorId::CAM0);
      config_->getSelectedCameraCalibration(&cam1_calibration, SensorId::SensorId::CAM1);
      dense_matcher->setCalibration( cam0_calibration, cam1_calibration);
    }
  }
}

void ViSensorDriver::Impl::selectCameraCalibration() {
  selectCameraCalibration(-1, -1, ViCameraLensModel::LensModelTypes::UNKNOWN, ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
}

void ViSensorDriver::Impl::getSelectedCameraCalibration(ViCameraCalibration* usedCalibration, const SensorId::SensorId camera_id) const {
  config_->getSelectedCameraCalibration(usedCalibration, camera_id);
}

void ViSensorDriver::Impl::sendSerialData(ViSerialData::Ptr data)
{
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  //send the data
  ip_connection_->sendSerialData(data);
}

void ViSensorDriver::Impl::setSerialCallback(boost::function<void(ViSerialData::Ptr)> callback)
{
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  //register callback
  serial_host_->setSerialDataCallback(callback);
}

void ViSensorDriver::Impl::setSerialDelimiter(const char serial_id, const std::string delimiter)
{
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  ip_connection_->setSerialDelimiter(serial_id, delimiter);
}

void ViSensorDriver::Impl::setSerialBaudrate(const char serial_id, const unsigned int baudrate)
{
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  ip_connection_->setSerialBaudrate(serial_id, baudrate);
}

void ViSensorDriver::Impl::uploadFile(std::string& local_path,
                                        std::string& remote_path) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  ip_connection_->file_transfer()->uploadFile(local_path, remote_path);
}

void ViSensorDriver::Impl::setCornerCallback(
    boost::function<void(ViCorner::Ptr, ViErrorCode)> callback) {

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CORNER_MT9V034)
      it->second->setCornerCallback(callback);
  }
}

void ViSensorDriver::Impl::setFramesCornersCallback(
    boost::function<void(ViFrame::Ptr, ViCorner::Ptr)> callback) {

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CORNER_MT9V034) {

      // check if corresponding camera is available
      const SensorId::SensorId& corner_id = it->first;
      const SensorId::SensorId& camera_id = SensorId::getCamId(corner_id);

      if (sensors_.count(camera_id) == 0) {
        VISENSOR_DEBUG("Camera %d not available to sync corners %d with.\n", camera_id,
               it->first);
        continue;
      }

      VISENSOR_DEBUG("start camera corner synchronizer for cam %d", corner_id);

      cam_corner_syncronizer_.push_back(new FrameCornerSynchronizer());

      // register all the callbacks
      cam_corner_syncronizer_[cam_corner_syncronizer_.size() - 1]
          ->setUserCallback(callback);
      sensors_.at(corner_id)->setCornerCallback(
          boost::bind(
              &FrameCornerSynchronizer::addCorner,
              cam_corner_syncronizer_[cam_corner_syncronizer_.size() - 1], _1));
      sensors_.at(camera_id)->setFrameCallback(
          boost::bind(
              &FrameCornerSynchronizer::addFrame,
              cam_corner_syncronizer_[cam_corner_syncronizer_.size() - 1], _1));
    }
  }
}

void ViSensorDriver::Impl::startAllCorners() {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CORNER_MT9V034)
      startSensor(it->first, 1);
  }
}

std::vector<ViCameraCalibration> ViSensorDriver::Impl::getCameraCalibrations(const SensorId::SensorId cam_id) const {
  return getCameraCalibrations(cam_id, -1, -1, ViCameraLensModel::LensModelTypes::UNKNOWN, ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
}

std::vector<ViCameraCalibration> ViSensorDriver::Impl::getCameraCalibrations(const SensorId::SensorId cam_id,
                                                                             const int slot_id) const {
  return getCameraCalibrations(cam_id, slot_id, -1, ViCameraLensModel::LensModelTypes::UNKNOWN, ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
}

std::vector<ViCameraCalibration> ViSensorDriver::Impl::getCameraCalibrations(const SensorId::SensorId cam_id,
                                                                             const ViCameraLensModel::LensModelTypes lens_model_type,
                                                                             const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const {
  return getCameraCalibrations(cam_id, -1, -1, lens_model_type, projection_model_type);
}

std::vector<ViCameraCalibration> ViSensorDriver::Impl::getCameraCalibrations(const SensorId::SensorId cam_id,
                                                                             const int slot_id,
                                                                             const ViCameraLensModel::LensModelTypes lens_model_type,
                                                                             const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const {
  return getCameraCalibrations(cam_id, slot_id, -1, lens_model_type, projection_model_type);
}

std::vector<ViCameraCalibration> ViSensorDriver::Impl::getCameraCalibrations(const SensorId::SensorId cam_id,
                                                                             const int slot_id,
                                                                             const int is_flipped,
                                                                             const ViCameraLensModel::LensModelTypes lens_model_type,
                                                                             const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");
  if (!config_->isValid()) {
    config_->loadConfig();
  }
  return config_->getCameraCalibrations(cam_id, slot_id, is_flipped, lens_model_type, projection_model_type);
}

//Internally, we divide up the slots for flipped cameras (uneven slot_ids) and unflipped cameras (even slot_ids)
void ViSensorDriver::Impl::setCameraCalibration(const ViCameraCalibration& calib){
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  if (config_->isValid() != true) {
    throw visensor::exceptions::ConfigException("Load general calibration first before setting any calibration to avoid overwite of existing configuration");
  }
  //slot 0 holds the factory calibration and can't be overwritten using the public API
  int slot_id = calib.slot_id_;
  if (slot_id == 0){
    throw visensor::exceptions::ConfigException("Slot ID 0 is reserved for the factory calibrations");
  }

  if (!config_->setCameraCalibration(calib)) {
    throw visensor::exceptions::ConfigException("Failed to set camera calibration.");
  }
  if (!config_->saveConfig()) {
    throw visensor::exceptions::ConnectionException("Failed to store the configuration on the sensor");
  }
}

//Check if stereo camera is flipped
//We assume that cam0 & cam1 are in stereo configuration
bool ViSensorDriver::Impl::isStereoCameraFlipped()
{
  bool is_cam0_flipped = false;
  bool is_cam1_flipped = false;

  try {
    ViCameraCalibration calibration;
    config_->getSelectedCameraCalibration(&calibration, SensorId::SensorId::CAM0);
    is_cam0_flipped = calibration.is_flipped_;
    config_->getSelectedCameraCalibration(&calibration, SensorId::SensorId::CAM1);
    is_cam1_flipped = calibration.is_flipped_;
  }
  catch (const std::exception& e)
  {
    VISENSOR_DEBUG("ViSensorDriver::Impl: Could not find set calibrations\n");
  }
  return (is_cam0_flipped &&  is_cam1_flipped);
}

bool ViSensorDriver::Impl::cleanCameraCalibrations(const SensorId::SensorId cam_id,
                                                   const int slot_id,
                                                   const int is_flipped,
                                                   const ViCameraLensModel::LensModelTypes lens_model_type,
                                                   const ViCameraProjectionModel::ProjectionModelTypes projection_model_type){
  if(config_->cleanCameraCalibration(cam_id, slot_id, is_flipped, lens_model_type, projection_model_type)) {
    config_->saveConfig();
    return true;
  }
  return false;
}

bool ViSensorDriver::Impl::cleanCameraCalibrations(const SensorId::SensorId cam_id, const int slot_id){
  return cleanCameraCalibrations(cam_id, slot_id, -1, ViCameraLensModel::LensModelTypes::UNKNOWN, ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
}

bool ViSensorDriver::Impl::cleanCameraCalibrations(const SensorId::SensorId cam_id){
  return cleanCameraCalibrations(cam_id, -1, -1, ViCameraLensModel::LensModelTypes::UNKNOWN, ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
}

bool ViSensorDriver::Impl::cleanCameraCalibrations(){
  return cleanCameraCalibrations( SensorId::SensorId::NOT_SPECIFIED, -1, -1, ViCameraLensModel::LensModelTypes::UNKNOWN, ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
}

//set factory calibration on slot 0 (private API call)
//
void ViSensorDriver::Impl::setCameraFactoryCalibration(const ViCameraCalibration calib){
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  if (config_->isValid() != true) {
    throw visensor::exceptions::ConfigException("Load general calibration first before setting any calibration to avoid overwite of existing configuration");
  }
  if(calib.slot_id_ != 0){
    throw visensor::exceptions::ConfigException("Slot ID has to be 0 for the factory calibrations.");
  }

  if (!config_->setCameraCalibration(calib)) {
    throw visensor::exceptions::ConfigException("Failed to set factory camera calibration.");
  }
  if (!config_->saveConfig()) {
    throw visensor::exceptions::ConnectionException("Failed to store the configuration on the sensor");
  }
}

bool ViSensorDriver::Impl::isSensorPresent(const SensorId::SensorId sensor_id) const {
  bool isPresent = false;
  if(sensors_.count(sensor_id) > 0)
    isPresent = true;
  return isPresent;
}

void ViSensorDriver::Impl::setViSensorId(const int vi_sensor_id) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");
  if (!config_->isValid()) {
    config_->loadConfig();
  }
  if (!config_->setViSensorId(vi_sensor_id))
    throw visensor::exceptions::ConfigException("Failed to set the VISensor ID!");

  if (!config_->saveConfig()) {
    throw visensor::exceptions::ConnectionException("Failed to store the configuration on the sensor");
  }
}

int ViSensorDriver::Impl::getViSensorId() const {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");
  if (!config_->isValid()) {
    config_->loadConfig();
  }
  int sensor_id;
  if ((sensor_id = config_->getViSensorId()) <  0)
    throw visensor::exceptions::ConfigException("Failed to get the VISensor ID!");
  return sensor_id;
}

void ViSensorDriver::Impl::getUserConfiguration(const std::string& key, int* value) const
{
  if (!initialized_)
    throw visensor::exceptions::ConnectionException("No connection to sensor.");
  if (!config_->isValid()) {
    config_->loadConfig();
  }
  config_->getUserConfiguration(key, value);
}

void ViSensorDriver::Impl::getUserConfiguration(const std::string& key,
                                                    std::string* value) const
{
  if (!initialized_)
    throw visensor::exceptions::ConnectionException("No connection to sensor.");
  if (!config_->isValid()) {
    config_->loadConfig();
  }
  config_->getUserConfiguration(key, value);
}

void ViSensorDriver::Impl::setUserConfiguration(const std::string& key, const int& value)
{
  if (!initialized_)
    throw visensor::exceptions::ConnectionException("No connection to sensor.");
  if (!config_->isValid()) {
    config_->loadConfig();
  }
  config_->setUserConfiguration(key, value);

  if (!config_->saveConfig()) {
    throw visensor::exceptions::ConnectionException(
        "Failed to store the configuration on the sensor");
  }
}

void ViSensorDriver::Impl::setUserConfiguration(const std::string& key,
                                                    const std::string& value)
{
  if (!initialized_)
    throw visensor::exceptions::ConnectionException("No connection to sensor.");
  if (!config_->isValid()) {
    config_->loadConfig();
  }
  config_->setUserConfiguration(key, value);

  if (!config_->saveConfig()) {
    throw visensor::exceptions::ConnectionException(
        "Failed to store the configuration on the sensor");
  }
}

}  //namespace visensor
