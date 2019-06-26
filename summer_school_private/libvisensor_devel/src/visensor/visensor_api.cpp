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

#include "visensor/visensor_api.hpp"
#include "visensor_impl.hpp"

namespace visensor
{

ViSensorDriver::ViSensorDriver()
{
  pImpl_ = new Impl();
}

void ViSensorDriver::getAutoDiscoveryDeviceList(ViDeviceList &hostname_list)
{
  pImpl_->getAutoDiscoveryDeviceList(hostname_list);
}

std::string ViSensorDriver::init()
{
  return pImpl_->initAutodiscovery();
}

void ViSensorDriver::init(std::string hostname)
{
  pImpl_->init(hostname);
}

ViSensorDriver::~ViSensorDriver()//:_fpga(&getSensorFromID){
{
  delete pImpl_;
}

void ViSensorDriver::setCameraCallback(boost::function<void (ViFrame::Ptr, ViErrorCode)> callback)
{
  pImpl_->setCameraCallback(callback);
}

void ViSensorDriver::setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback) {
  pImpl_->setCornerCallback(callback);
}

void ViSensorDriver::setFramesCornersCallback(boost::function<void(ViFrame::Ptr, ViCorner::Ptr)> callback) {
  pImpl_->setFramesCornersCallback(callback);
}

void ViSensorDriver::setImuCallback(boost::function<void (boost::shared_ptr<ViImuMsg>, ViErrorCode)> callback)
{
  pImpl_->setImuCallback(callback);
}

void ViSensorDriver::startAllCameras(uint32_t rate) {
  pImpl_->startAllCameras(rate);
}

void ViSensorDriver::startSensor(SensorId::SensorId id, uint32_t rate)
{
  pImpl_->startSensor(id, rate);
}

void ViSensorDriver::stopSensor(SensorId::SensorId id)
{
  pImpl_->stopSensor(id);
}

void ViSensorDriver::setSensorConfigParam(SensorId::SensorId sensor_id,
                                                  std::string cmd,
                                                  int value) {
  pImpl_->setSensorConfigParam(sensor_id, cmd, value);
}

void ViSensorDriver::startAllCorners() {
  pImpl_->startAllCorners();
}

void ViSensorDriver::startAllImus(uint32_t rate) {
  pImpl_->startAllImus(rate);
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfSensorIDs() const
{
  return pImpl_->getListOfSensorIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfCameraIDs() const
{
	return pImpl_->getListOfCameraIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfDenseIDs() const
{
  return pImpl_->getListOfDenseIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfCornerIDs() const {
  return pImpl_->getListOfCornerIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfImuIDs() const
{
	return pImpl_->getListOfImuIDs();
}

std::vector<SensorId::SensorId> ViSensorDriver::getListOfTriggerIDs() const
{
	return pImpl_->getListOfTriggerIDs();
}

uint32_t ViSensorDriver::getFpgaId() const
{
	return pImpl_->getFpgaId();
}

void ViSensorDriver::downloadFile(std::string local_path, std::string remote_path) {
  pImpl_->downloadFile(local_path, remote_path);
}

void ViSensorDriver::uploadFile(std::string local_path, std::string remote_path) {
  pImpl_->uploadFile(local_path, remote_path);
}

void ViSensorDriver::startAllExternalTriggers(uint32_t rate) {
  pImpl_->startAllExternalTriggers(rate);
}

void ViSensorDriver::setExternalTriggerCallback(
    boost::function<void(ViExternalTriggerMsg::Ptr)> callback) {
  pImpl_->setExternalTriggerCallback(callback);
}

void ViSensorDriver::setExternalTriggerConfig(const ViExternalTriggerConfig config) {
  pImpl_->setExternalTriggerConfig(config);
}

void ViSensorDriver::startAllDenseMatchers() {
  pImpl_->startAllDenseMatchers();
}

void ViSensorDriver::setDenseMatcherCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback) {
  pImpl_->setDenseMatcherCallback(callback);
}

void ViSensorDriver::selectCameraCalibration(const SensorId::SensorId cam_id,
                                               const int slot_id,
                                               const int is_flipped,
                                               const ViCameraLensModel::LensModelTypes lens_model_type,
                                               const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) {
  pImpl_->selectCameraCalibration(cam_id, slot_id, is_flipped, lens_model_type, projection_model_type);
}

void ViSensorDriver::selectCameraCalibration() {
  pImpl_->selectCameraCalibration();
}

void ViSensorDriver::getSelectedCameraCalibration(ViCameraCalibration* usedCalibration, const SensorId::SensorId camera_id) const {
  pImpl_->getSelectedCameraCalibration(usedCalibration, camera_id);
}

std::vector<ViCameraCalibration> ViSensorDriver::getCameraCalibrations(SensorId::SensorId cam_id) const {
  return pImpl_->getCameraCalibrations(cam_id);
}

std::vector<ViCameraCalibration> ViSensorDriver::getCameraCalibrations(SensorId::SensorId cam_id,
                                                                       int slot_id) const {
  return pImpl_->getCameraCalibrations(cam_id, slot_id);
}

std::vector<ViCameraCalibration> ViSensorDriver::getCameraCalibrations(SensorId::SensorId cam_id,
                                                                       ViCameraLensModel::LensModelTypes lens_model_type,
                                                                       ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const {
  return pImpl_->getCameraCalibrations(cam_id, lens_model_type, projection_model_type);
}

std::vector<ViCameraCalibration> ViSensorDriver::getCameraCalibrations(SensorId::SensorId cam_id,
                                                                       int slot_id,
                                                                       ViCameraLensModel::LensModelTypes lens_model_type,
                                                                       ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const {
  return pImpl_->getCameraCalibrations(cam_id, slot_id, lens_model_type, projection_model_type);
}

std::vector<ViCameraCalibration> ViSensorDriver::getCameraCalibrations(SensorId::SensorId cam_id,
                                                                       int slot_id,
                                                                       int is_flipped,
                                                                       ViCameraLensModel::LensModelTypes lens_model_type,
                                                                       ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const {
  return pImpl_->getCameraCalibrations(cam_id, slot_id, is_flipped, lens_model_type, projection_model_type);
}

void ViSensorDriver::setCameraCalibration(const ViCameraCalibration& calib) const
{
  pImpl_->setCameraCalibration(calib);
}

bool ViSensorDriver::cleanCameraCalibrations(const SensorId::SensorId cam_id,
                                                   const int slot_id,
                                                   const int is_flipped,
                                                   const ViCameraLensModel::LensModelTypes lens_model_type,
                                                   const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) {
  if(slot_id == 0){
    throw visensor::exceptions::ConfigException("The factory calibrations can not be cleaned.");
  }
  return pImpl_->cleanCameraCalibrations(cam_id, slot_id, is_flipped, lens_model_type, projection_model_type);
}

bool ViSensorDriver::cleanCameraCalibrations(const SensorId::SensorId cam_id){
  std::vector<ViCameraCalibration> calibrations = pImpl_->getCameraCalibrations(cam_id);
  bool deleted_one = false;
  for (std::vector<ViCameraCalibration>::iterator it = calibrations.begin();  it != calibrations.end(); ++it) {
    //ignore all factory calibrations. Use the impl class to delete the factory calibration
    if (it->slot_id_ == 0)
      continue;
    deleted_one = true;
    pImpl_->cleanCameraCalibrations(cam_id, it->slot_id_, it->is_flipped_, it->lens_model_->type_, it->projection_model_->type_);
  }
  return deleted_one;
}

bool ViSensorDriver::cleanCameraCalibrations(){
  return cleanCameraCalibrations( SensorId::SensorId::NOT_SPECIFIED);
}

void ViSensorDriver::setViSensorId(const int vi_sensor_id) {
  pImpl_->setViSensorId(vi_sensor_id);
}

int ViSensorDriver::getViSensorId() const {
  return pImpl_->getViSensorId();
}

void ViSensorDriver::getUserConfiguration(const std::string& key, std::string* value) const
{
  pImpl_->getUserConfiguration(key, value);
}

void ViSensorDriver::getUserConfiguration(const std::string& key, int* value) const
{
  pImpl_->getUserConfiguration(key, value);
}

void ViSensorDriver::setUserConfiguration(const std::string& key, const std::string& value)
{
  pImpl_->setUserConfiguration(key, value);
}

void ViSensorDriver::setUserConfiguration(const std::string& key, const int& value)
{
  pImpl_->setUserConfiguration(key, value);
}

bool ViSensorDriver::isStereoCameraFlipped() const
{
  return pImpl_->isStereoCameraFlipped();
}


void ViSensorDriver::sendSerialData(ViSerialData::Ptr data) const
{
  pImpl_->sendSerialData(data);
}

void ViSensorDriver::setSerialCallback(boost::function<void(ViSerialData::Ptr)> callback)
{
  pImpl_->setSerialCallback(callback);
}

void ViSensorDriver::setSerialDelimiter(const char serial_id, const std::string delimiter) const
{
  pImpl_->setSerialDelimiter(serial_id, delimiter);
}

void ViSensorDriver::setSerialBaudrate(const char serial_id, const unsigned int baudrate) const
{
  pImpl_->setSerialBaudrate(serial_id, baudrate);
}

bool ViSensorDriver::isSensorPresent(const SensorId::SensorId sensor_id) const
{
  return pImpl_->isSensorPresent(sensor_id);
}

}  //namespace visensor
