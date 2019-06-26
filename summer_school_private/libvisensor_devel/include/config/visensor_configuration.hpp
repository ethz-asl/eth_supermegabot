/*
 * Copyright (c) 2015, Skybotix AG, Switzerland (info@skybotix.com)
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

#ifndef LIBVISENSOR_CONFIG_VISENSORCONFIG_H_
#define LIBVISENSOR_CONFIG_VISENSORCONFIG_H_

#include <memory>

#include <yaml-cpp/yaml.h>

#include "networking/connection.hpp"
#include "visensor/visensor.hpp"

namespace visensor {
/**
 * \class ViSensorConfig
 *
 * \brief Configuration class containing all the configuration of the Vi-Sensor, all cameras, imus, etc.
 * but also the calibrations of the cameras.
 *
 * \date May 15, 2015
 */
class DSO_EXPORT ViSensorConfiguration
{
 public:
  typedef boost::shared_ptr<ViSensorConfiguration> Ptr;
  typedef boost::weak_ptr<ViSensorConfiguration> WeakPtr;
  typedef std::map<SensorId::SensorId, ViCameraCalibration> CalibrationMap;

  /**
   * Enums for the corresponding YAML keys
   */
  enum class ConfigYaml_e
  {
    UNKOWN,
    CALIBRATION,
    SENSOR_ID,
    USER
  };
  enum class ConfigYamlCamera_e
  {
    UNKOWN,
    CALIBRATION,
    CAMERA,
    USED_CALIBRATION,
    CAM_NUMBER
  };
  enum class ConfigYamlCameraCalibration_e
  {
    UNKOWN,
    IS_FLIPPED,
    RESOLUTION,
    SLOT_ID,
    CAM_NUMBER,
    PROJECTION_MODEL,
    LENS_MODEL,
    R,
    T
  };
  enum class ConfigYamlLensModelStruct_e
  {
    UNKOWN,
    TYPE,
    COEFFICIENTS
  };
  enum class ConfigYamlCameraProjectionModelStruct_e
  {
    UNKOWN,
    TYPE,
    COEFFICIENTS
  };
  const std::string REMOTE_CONFIG_PATH;

  ViSensorConfiguration(FileTransfer::Ptr file_transfer);
  virtual ~ViSensorConfiguration();

  /**
   * Loads the configuration from the Vi-Sensor
   *
   * @return true if the loading was successful, otherwise false
   */
  void loadConfig();

  /**
   * Loads the configuration from a specific place.
   *
   * Depends on the file_transfere object has the file be on the vi_sensor or somewhere else
   *
   * @return true if the loading was successful, otherwise false
   */
  void loadConfig(const std::string& config_path);

  /**
   * Saves the configuration from the Vi-Sensor
   *
   * @return true if the saving was successful, otherwise false
   */
  bool saveConfig();

  /**
   * Saves the configuration to a specific place.
   *
   * Depends on the file_transfere object has the file be on the vi_sensor or somewhere else
   *
   * @return true if the saving was successful, otherwise false
   */
  bool saveConfig(const std::string& config_path);

  /**
   * Returns the saved Sensor ID.
   *
   * Load first the configuration with loadConfig()
   *
   * @return VISensor id if successful, -1 if failed
   */
  int  getViSensorId() const;

  /**
   * Sets the VISensor id without saving the configuration.
   *
   * To save the configuration use saveConfig.
   *
   * @return true if the seting was successful, otherwise false
   */
  bool setViSensorId(const int vi_sensor_id);

  /**
   * Gets user configuration parameters which were saved previous on the visensor
   * in case of an error an visensor exception (ConfigException) is thrown.
   *
   * @param key:    identifier for the parameter
   * @param value:  return value
   */
  void getUserConfiguration(const std::string& key, int* value );
  void getUserConfiguration(const std::string& key, std::string* value );

  /**
   * sets an user configuration parameter
   *
   * in case of an error an visensor exception (ConfigException) is thrown.
   *
   * @param key:    identifier for the parameter
   * @param value:  value to set
   */
  void setUserConfiguration(const std::string& key, const int& value );
  void setUserConfiguration(const std::string& key, const std::string& value );

  /**
   * returns the specific camera calibration.
   *
   * If more than one calibration configuration matches to the input values a vector is returned.
   *
   * @param cam_id:     filters the calibrations for sensor id (<0 means don't care)
   * @param slot_id:    defines the used slot. 0 is for factory calibration, >0 for customer. <0 means don't care
   * @param is_flipped: filters calibration for flipped or non flipped calibrations. < 0 means don't care
   * @param lens_model_type:  filters calibration for specific lens model type. set type to unknown if the lens model type does not care
   * @param projection_model_type:  filters calibration for specific projection model type. set type to unknown if the projection model  type does not care
   *
   * @return copy of a vector of matching calibrations
   */
  std::vector<ViCameraCalibration> getCameraCalibrations(
      const SensorId::SensorId cam_id, const int slot_id, const int is_flipped,
      const ViCameraLensModel::LensModelTypes lens_model_type,
      const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(
      const SensorId::SensorId cam_id, const int slot_id,
      const ViCameraLensModel::LensModelTypes lens_model_type,
      const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(
      const SensorId::SensorId cam_id,
      const int slot_id,
      const int is_flipped) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(
      const SensorId::SensorId cam_id,
      const int slot_id) const;
  std::vector<ViCameraCalibration> getCameraCalibrations(const SensorId::SensorId cam_id) const;
  std::vector<ViCameraCalibration> getCameraCalibrations() const;

  /**
   * returns the configured/ used calibration for the camera
   *
   * @param calibration returns the used calibration
   * @param cam_id:     sensor id
   *
   */
  void getSelectedCameraCalibration(ViCameraCalibration* usedCalibration,
                                const SensorId::SensorId camera_id) const;

  /**
   * configure/set which calibration is used for the camera
   *
   * @param cam_id:     sensor id
   *
   * @return copy of a vector of matching calibrations
   */
  void selectCameraCalibration(const ViCameraCalibration& usedCalibration,
                                const SensorId::SensorId camera_id);

  /**
   * delete specific camera calibration(s)
   *
   * @param cam_id:     filters the calibrations for sensor id (<0 means don't care)
   * @param slot_id:    defines the used slot. 0 is for factory calibration, >0 for customer. <0 means don't care
   * @param is_flipped: filters calibration for flipped or non flipped calibrations. < 0 means don't care
   * @param lens_model_type:  filters calibration for specific lens model type. set type to unknown if the lens model type does not care
   * @param projection_model_type:  filters calibration for specific projection model type. set type to unknown if the projection model  type does not care
   *
   * @return true if successful
   */
  bool cleanCameraCalibration(
      const SensorId::SensorId cam_id, const int slot_id, const int is_flipped,
      const ViCameraLensModel::LensModelTypes lens_model_type,
      const ViCameraProjectionModel::ProjectionModelTypes projection_model_type);

  /**
   * set the calibration local
   *
   * This function append the existing camera calibrations with the given if the parameters
   * cam_id,
   * slot_id,
   * is_flipped,
   * lens_model_type
   * and projection_model_type not maching to an existing one.
   * Otherwise the existing calibration is overwitten by the given object.
   * This function does not save the configuration on the device. Use the saveConfig for that.
   *
   * @param calibration:  calibration object to set.
   *
   * @return true if the parsing worked
   */
  bool setCameraCalibration(const ViCameraCalibration& calibration);

  /**
   * check if the config was loaded successfully
   *
   * @return true if the config is valid
   */
  bool isValid() const;

  /**
   * copy/move constructor is called to construct obj
   *
   * @return new configuration object
   */
  ViSensorConfiguration& operator=(const ViSensorConfiguration& obj);  // destructor is called to release the resources formerly held by *this

 private:
  /**
   * parse calibration class to yaml node
   *
   * @return true if the parsing worked
   */
  bool emitCameraCalibration();
  bool emitCameraCalibration(YAML::Node* node, const ViCameraCalibration& calibration);
  bool emitCameraCalibration(YAML::Node* node, const ViCameraLensModel::Ptr lens_model);
  bool emitCameraCalibration(YAML::Node* node, const ViCameraLensModelRadtan::Ptr lens_model);
  bool emitCameraCalibration(YAML::Node* node, const ViCameraLensModelEquidistant::Ptr lens_model);
  bool emitCameraCalibration(YAML::Node* node, const ViCameraProjectionModel::Ptr projection_model);
  bool emitCameraCalibration(YAML::Node* node,
                             const ViCameraProjectionModelOmnidirectional::Ptr projection_model);
  bool emitCameraCalibration(YAML::Node* node,
                             const ViCameraProjectionModelPinhole::Ptr projection_model);

  /**
   * parse yaml node to calibration class and save it as a member variable
   *
   * @return true if the parsing worked
   */
  std::vector<ViCameraCalibration> parseYaml(const YAML::Node& config_nodes);
  void parseYaml(const YAML::Node& node, ViCameraCalibration* calibration, int camera_number);
  void parseYaml(const YAML::Node& node, ViCameraLensModelRadtan::Ptr lens_model);
  void parseYaml(const YAML::Node& node, ViCameraLensModelEquidistant::Ptr lens_model);
  void parseYaml(const YAML::Node& node,
                 ViCameraProjectionModelOmnidirectional::Ptr projection_model);
  void parseYaml(const YAML::Node& node, ViCameraProjectionModelPinhole::Ptr projection_model);

  /**
   * Pointer to the file transfer class to load and save the config file
   */
  FileTransfer::Ptr file_transfer_;

  /**
   * internal configuration nodes
   */
  YAML::Node config_nodes_;

  /**
   * internal calibrations. Used to avoid parsing of the config_nodes every time
   */
  std::vector<ViCameraCalibration> config_calibrations_;

  /**
   * internal calibrations which were selected to run
   */
  CalibrationMap selectedCalibration_;

  /**
   * internal calibrations. Used to avoid parsing of the config_nodes every time
   */
  bool valid_;
};

/**
* maps with the YAML keys and the corresponding enum
*/
static const std::map<ViSensorConfiguration::ConfigYaml_e, std::string> yaml_naming_ = {
    { ViSensorConfiguration::ConfigYaml_e::SENSOR_ID, "sensorID"},
    { ViSensorConfiguration::ConfigYaml_e::USER, "user"},
    { ViSensorConfiguration::ConfigYaml_e::CALIBRATION, "Vi_camera_calibration" } };

static const std::map<ViSensorConfiguration::ConfigYamlCamera_e, std::string> yaml_camera_ = {
    { ViSensorConfiguration::ConfigYamlCamera_e::CALIBRATION, "calibration" },
    { ViSensorConfiguration::ConfigYamlCamera_e::CAMERA, "camera" },
    { ViSensorConfiguration::ConfigYamlCamera_e::CAM_NUMBER, "cam_number" } };

static const std::map<ViCameraLensModel::LensModelTypes, std::string> yaml_camera_calibration_lensmodel_type_ = {
    { ViCameraLensModel::LensModelTypes::RADTAN, "radial" },
    { ViCameraLensModel::LensModelTypes::EQUIDISTANT, "equidistant" } };

static const std::map<ViSensorConfiguration::ConfigYamlLensModelStruct_e, std::string> yaml_camera_calibration_lensmodel_ = {
    { ViSensorConfiguration::ConfigYamlLensModelStruct_e::TYPE, "type" },
    { ViSensorConfiguration::ConfigYamlLensModelStruct_e::COEFFICIENTS, "coefficients" } };

static const std::map<ViCameraProjectionModel::ProjectionModelTypes, std::string> yaml_camera_calibration_projectionmodel_type_ = {
    { ViCameraProjectionModel::ProjectionModelTypes::PINHOLE, "pinhole" },
    { ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL, "omnidirectional" } };

static const std::map<ViSensorConfiguration::ConfigYamlCameraProjectionModelStruct_e, std::string> yaml_camera_calibration_projectionmodel_ = {
    { ViSensorConfiguration::ConfigYamlCameraProjectionModelStruct_e::TYPE, "type" },
    { ViSensorConfiguration::ConfigYamlCameraProjectionModelStruct_e::COEFFICIENTS, "coefficients" } };

static const std::map<ViSensorConfiguration::ConfigYamlCameraCalibration_e, std::string> yaml_camera_calibration_ = {
    { ViSensorConfiguration::ConfigYamlCameraCalibration_e::IS_FLIPPED, "is_flipped" },
    { ViSensorConfiguration::ConfigYamlCameraCalibration_e::RESOLUTION, "resolution" },
    { ViSensorConfiguration::ConfigYamlCameraCalibration_e::SLOT_ID, "slot_id" },
    { ViSensorConfiguration::ConfigYamlCameraCalibration_e::CAM_NUMBER, "cam_number" },
    { ViSensorConfiguration::ConfigYamlCameraCalibration_e::LENS_MODEL, "lens_model" },
    { ViSensorConfiguration::ConfigYamlCameraCalibration_e::PROJECTION_MODEL, "projection_model" },
    { ViSensorConfiguration::ConfigYamlCameraCalibration_e::R, "R" },
    { ViSensorConfiguration::ConfigYamlCameraCalibration_e::T, "t" } };

  const std::string file_header_ =
      "#\n"
      "# Copyright (c) 2015, Skybotix AG, Switzerland (info@skybotix.com)\n"
      "#\n"
      "# All rights reserved.\n"
      "#\n"
      "# Licensed under the Apache License, Version 2.0 (the \"License\");\n"
      "# you may not use this file except in compliance with the License.\n"
      "# You may obtain a copy of the License at\n"
      "#\n"
      "# http://www.apache.org/licenses/LICENSE-2.0\n"
      "#\n"
      "# Unless required by applicable law or agreed to in writing, software\n"
      "# distributed under the License is distributed on an \"AS IS\" BASIS,\n"
      "# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
      "# See the License for the specific language governing permissions and\n"
      "# limitations under the License.\n"
      "#\n"
      "# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\" AND\n"
      "# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED\n"
      "# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE\n"
      "# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR\n"
      "# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES\n"
      "# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;\n"
      "# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON\n"
      "# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT\n"
      "# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS\n"
      "# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n"
      "#\n";
}
#endif /* LIBVISENSOR_CONFIG_VISENSORCONFIG_H_ */
