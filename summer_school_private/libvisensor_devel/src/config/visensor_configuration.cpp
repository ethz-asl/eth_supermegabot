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

#include "config/visensor_configuration.hpp"
#include "visensor/visensor_exceptions.hpp"

namespace visensor {
ViSensorConfiguration::ViSensorConfiguration(FileTransfer::Ptr file_transfer)
    : REMOTE_CONFIG_PATH("/home/root/config/config.yaml"),
      file_transfer_(file_transfer),
      valid_(false)
{
}

ViSensorConfiguration::~ViSensorConfiguration()
{
}

void ViSensorConfiguration::loadConfig()
{
  loadConfig(REMOTE_CONFIG_PATH);
}

void ViSensorConfiguration::loadConfig(const std::string& config_path)
{
  std::string file_content;
  try {
    file_transfer_->readRemoteFile(config_path, &file_content);
    config_nodes_ = YAML::Load(file_content);
    config_calibrations_ = parseYaml(config_nodes_);
    valid_ = true;
  } catch (visensor::exceptions const &ex) {
    throw visensor::exceptions::ConfigException(
        "Could not load the configuration from the sensor!");
  } catch (YAML::Exception &ex) {
    throw visensor::exceptions::ConfigException(
        "Could not parse the configuration from the sensor! Exception was " + std::string(ex.what()));
  }
}

bool ViSensorConfiguration::saveConfig()
{
  return saveConfig(REMOTE_CONFIG_PATH);
}

bool ViSensorConfiguration::saveConfig(const std::string& config_path)
{
  YAML::Emitter out;
  out << config_nodes_ << YAML::Newline;
  std::string output(file_header_ + out.c_str());
  file_transfer_->writeRemoteFile(config_path, output, true);
  return true;
}

bool ViSensorConfiguration::setViSensorId(const int vi_sensor_id) {
  config_nodes_[yaml_naming_.at(ConfigYaml_e::SENSOR_ID)] = vi_sensor_id;
  return true;
}

int ViSensorConfiguration::getViSensorId() const {
  if (!isValid())
    return -1;
  if (config_nodes_[yaml_naming_.at(ConfigYaml_e::SENSOR_ID)])
    return config_nodes_[yaml_naming_.at(ConfigYaml_e::SENSOR_ID)].as<int>();
  return -1;
}

void ViSensorConfiguration::getUserConfiguration(const std::string& key, int* value)
{
  if (key.size() == 0)
    throw visensor::exceptions::ConfigException("No key is provided");

  YAML::Node costumer_node;
  if (!config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)])
    throw visensor::exceptions::ConfigException("No user configuration were saved");

  costumer_node = config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)];

  if (!costumer_node[key])
    throw visensor::exceptions::ConfigException("Key " + key + " was not found");
  *value = costumer_node[key].as<int>();
}

void ViSensorConfiguration::getUserConfiguration(const std::string& key, std::string* value)
{
  if (key.size() == 0)
    throw visensor::exceptions::ConfigException("No key is provided");

  YAML::Node costumer_node;
  if (!config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)])
    throw visensor::exceptions::ConfigException("No user configuration were saved");

  costumer_node = config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)];

  if (!costumer_node[key])
    throw visensor::exceptions::ConfigException("Key " + key + " was not found");
  *value = costumer_node[key].as<std::string>();
}

void ViSensorConfiguration::setUserConfiguration(const std::string& key, const int& value)
{
  if (key.size() == 0)
    throw visensor::exceptions::ConfigException("No key is provided");

  YAML::Node costumer_node;
  if (config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)])
    costumer_node = config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)];
  else
    config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)] = costumer_node;
  costumer_node[key] = value;
}

void ViSensorConfiguration::setUserConfiguration(const std::string& key,
                                                     const std::string& value)
{
  if (key.size() == 0)
    throw visensor::exceptions::ConfigException("No key is provided");

  YAML::Node costumer_node;
  if (config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)])
    costumer_node = config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)];
  else
    config_nodes_[yaml_naming_.at(ConfigYaml_e::USER)] = costumer_node;
  costumer_node[key] = value;
}

bool ViSensorConfiguration::cleanCameraCalibration(
    const SensorId::SensorId cam_id, const int slot_id, const int is_flipped,
    const ViCameraLensModel::LensModelTypes lens_model_type,
    const ViCameraProjectionModel::ProjectionModelTypes projection_model_type)
{
  bool deleted_one = false;
  config_calibrations_.erase(
      std::remove_if(
          config_calibrations_.begin(),
          config_calibrations_.end(),
          [cam_id, slot_id, is_flipped, lens_model_type, projection_model_type, &deleted_one]
          (const ViCameraCalibration& calibration)->bool
          {
            if ((calibration.slot_id_ != slot_id) && (slot_id >= 0))
              return false;
            if ((calibration.cam_id_ != static_cast<int>(cam_id)) && (static_cast<int>(cam_id) >= 0))
              return false;
            if ((calibration.is_flipped_ != is_flipped) && (is_flipped >= 0))
              return false;
            if ((calibration.lens_model_->type_ != lens_model_type) && (lens_model_type != ViCameraLensModel::LensModelTypes::UNKNOWN))
              return false;
            if ((calibration.projection_model_->type_ != projection_model_type) && (projection_model_type != ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN))
              return false;

            deleted_one = true;
            return true;
          }),
      config_calibrations_.end());
  emitCameraCalibration();
  return deleted_one;
}

std::vector<ViCameraCalibration> ViSensorConfiguration::getCameraCalibrations() const
{
  std::vector<ViCameraCalibration> output_vector = config_calibrations_;
  return output_vector;
}

std::vector<ViCameraCalibration> ViSensorConfiguration::getCameraCalibrations(
    const SensorId::SensorId cam_id) const
{
  return getCameraCalibrations(cam_id, -1, -1, ViCameraLensModel::LensModelTypes::UNKNOWN,
                               ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
}

std::vector<ViCameraCalibration> ViSensorConfiguration::getCameraCalibrations(
    const SensorId::SensorId cam_id, const int slot_id) const
{
  return getCameraCalibrations(cam_id, slot_id, -1, ViCameraLensModel::LensModelTypes::UNKNOWN,
                               ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
}

std::vector<ViCameraCalibration> ViSensorConfiguration::getCameraCalibrations(
    const SensorId::SensorId cam_id, const int slot_id, const int is_flipped) const
{
  return getCameraCalibrations(cam_id, slot_id, is_flipped,
                               ViCameraLensModel::LensModelTypes::UNKNOWN,
                               ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
}

std::vector<ViCameraCalibration> ViSensorConfiguration::getCameraCalibrations(
    const SensorId::SensorId cam_id, const int slot_id,
    const ViCameraLensModel::LensModelTypes lens_model_type,
    const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const
{
  return getCameraCalibrations(cam_id, slot_id, -1, lens_model_type, projection_model_type);
}

std::vector<ViCameraCalibration> ViSensorConfiguration::getCameraCalibrations(
    const SensorId::SensorId cam_id, const int slot_id, const int is_flipped,
    const ViCameraLensModel::LensModelTypes lens_model_type,
    const ViCameraProjectionModel::ProjectionModelTypes projection_model_type) const
{
  std::vector<ViCameraCalibration> output_vector;
  for (const ViCameraCalibration& calibration : config_calibrations_) {
    if ((calibration.slot_id_ != slot_id) && (slot_id >= 0))
      continue;
    if ((calibration.cam_id_ != static_cast<int>(cam_id)) && (static_cast<int>(cam_id) >= 0))
      continue;
    if ((calibration.is_flipped_ != is_flipped) && (is_flipped >= 0))
      continue;
    if ((calibration.lens_model_->type_ != lens_model_type)
        && (lens_model_type != ViCameraLensModel::LensModelTypes::UNKNOWN))
      continue;
    if ((calibration.projection_model_->type_ != projection_model_type)
        && (projection_model_type != ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN))
      continue;
    output_vector.push_back(calibration);
  }
  if (output_vector.empty())
    VISENSOR_DEBUG("NO Camera Calibration found. cam_id: %d slot_id: %d, is_flipped: %d\n", cam_id,
                   slot_id, is_flipped);
  return output_vector;
}

void ViSensorConfiguration::getSelectedCameraCalibration(ViCameraCalibration* usedCalibration,
                                                     const SensorId::SensorId camera_id) const
{
  try{
    *usedCalibration = selectedCalibration_.at(camera_id);
  } catch (const std::exception & ex) {
    throw visensor::exceptions::ConfigException(
        "could not find selected calibration for camera " + std::to_string(camera_id));
  }
}

void ViSensorConfiguration::selectCameraCalibration(const ViCameraCalibration& calibration,
                                                     const SensorId::SensorId camera_id)
{
  selectedCalibration_.insert( { camera_id, calibration });
}

bool ViSensorConfiguration::setCameraCalibration(const ViCameraCalibration& calibration_to_set)
{
  for (ViCameraCalibration& calibration : config_calibrations_) {
    if (calibration.slot_id_ != calibration_to_set.slot_id_)
      continue;
    if (calibration.cam_id_ != calibration_to_set.cam_id_)
      continue;
    if (calibration.is_flipped_ != calibration_to_set.is_flipped_)
      continue;
    if ((calibration.lens_model_->type_ != calibration_to_set.lens_model_->type_))
      continue;
    if ((calibration.projection_model_->type_ != calibration_to_set.projection_model_->type_))
      continue;
    calibration = calibration_to_set;
    return emitCameraCalibration();
  }
  config_calibrations_.push_back(calibration_to_set);
  return emitCameraCalibration();
}

bool ViSensorConfiguration::emitCameraCalibration()
{
  YAML::Node main_camera_nodes;
  config_nodes_[yaml_naming_.at(ConfigYaml_e::CALIBRATION)] = main_camera_nodes;
  for (const ViCameraCalibration& calibration : config_calibrations_) {
    YAML::Node calibration_node;
    if (!emitCameraCalibration(&calibration_node, calibration)) {
      return false;
    }

    // add to existing camera node if available
    bool new_cam_node = true;

    for (YAML::Node&& camera_nodes : main_camera_nodes) {
      YAML::Node cam_node = camera_nodes[yaml_camera_.at(ConfigYamlCamera_e::CAMERA)];
      if (cam_node[yaml_camera_.at(ConfigYamlCamera_e::CAM_NUMBER)].as<int>() == calibration.cam_id_) {
        cam_node[yaml_camera_.at(ConfigYamlCamera_e::CALIBRATION)].push_back(calibration_node);
        new_cam_node = false;
        break;
      }
    }
    if (new_cam_node == true) {
      YAML::Node cam_node_new;
      YAML::Node camera_nodes;
      camera_nodes[yaml_camera_.at(ConfigYamlCamera_e::CAMERA)] = cam_node_new;
      cam_node_new[yaml_camera_.at(ConfigYamlCamera_e::CAM_NUMBER)] = calibration.cam_id_;
      cam_node_new[yaml_camera_.at(ConfigYamlCamera_e::CALIBRATION)].push_back(calibration_node);
      main_camera_nodes.push_back(camera_nodes);
    }
  }
  return true;
}

bool ViSensorConfiguration::emitCameraCalibration(YAML::Node* calibration_node,
                                                  const ViCameraCalibration& calibration)
{
  YAML::Node lens_model_node;
  YAML::Node projection_model_node;
  emitCameraCalibration(&lens_model_node, calibration.lens_model_);
  (*calibration_node)[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::LENS_MODEL)] =
      lens_model_node;
  emitCameraCalibration(&projection_model_node, calibration.projection_model_);
  (*calibration_node)[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::PROJECTION_MODEL)] =
      projection_model_node;
  (*calibration_node)[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::SLOT_ID)] =
      calibration.slot_id_;
  (*calibration_node)[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::IS_FLIPPED)] =
      calibration.is_flipped_;
  for (unsigned int i = 0; i < ViCameraCalibration::NUMBER_OF_RESOLUTION; i++) {
    (*calibration_node)[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::RESOLUTION)]
        .push_back(calibration.resolution_[i]);
  }
  if (calibration.t_.size() != ViCameraCalibration::NUMBER_OF_T) {
    VISENSOR_DEBUG("calibration struct has to few entries of t\n");
    return false;
  }
  for (unsigned int i = 0; i < ViCameraCalibration::NUMBER_OF_T; i++) {
    (*calibration_node)[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::T)].push_back(
        calibration.t_[i]);
  }
  if (calibration.R_.size() != ViCameraCalibration::NUMBER_OF_R) {
    VISENSOR_DEBUG("calibration struct has to few entries of R\n");
    return false;
  }
  for (unsigned int i = 0; i < ViCameraCalibration::NUMBER_OF_R; i++) {
    (*calibration_node)[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::R)].push_back(
        calibration.R_[i]);
  }
  return true;
}

bool ViSensorConfiguration::emitCameraCalibration(YAML::Node* node,
                                                  const ViCameraLensModel::Ptr lens_model)
{
  (*node)[yaml_camera_calibration_lensmodel_.at(ConfigYamlLensModelStruct_e::TYPE)] =
      yaml_camera_calibration_lensmodel_type_.at(lens_model->type_);
  YAML::Node node_coefficients;
  switch (lens_model->type_) {
    case ViCameraLensModel::LensModelTypes::RADTAN:
      emitCameraCalibration(&node_coefficients,
                            std::static_pointer_cast<ViCameraLensModelRadtan>(lens_model));
      break;
    case ViCameraLensModel::LensModelTypes::EQUIDISTANT:
      emitCameraCalibration(&node_coefficients,
                            std::static_pointer_cast<ViCameraLensModelEquidistant>(lens_model));
      break;
    default:
      emitCameraCalibration(&node_coefficients,
                            std::static_pointer_cast<ViCameraLensModelRadtan>(lens_model));
      VISENSOR_DEBUG("Projection Model not known");
  }
  (*node)[yaml_camera_calibration_lensmodel_.at(ConfigYamlLensModelStruct_e::COEFFICIENTS)] =
      node_coefficients;

  return true;
}

bool ViSensorConfiguration::emitCameraCalibration(YAML::Node* node,
                                                  const ViCameraLensModelRadtan::Ptr lens_model)
{
  (*node)[static_cast<int>(ViCameraLensModelRadtan::RadtanCoefficients::K1)] = lens_model->k1_;
  (*node)[static_cast<int>(ViCameraLensModelRadtan::RadtanCoefficients::K2)] = lens_model->k2_;
  (*node)[static_cast<int>(ViCameraLensModelRadtan::RadtanCoefficients::R1)] = lens_model->r1_;
  (*node)[static_cast<int>(ViCameraLensModelRadtan::RadtanCoefficients::R2)] = lens_model->r2_;
  return true;
}

bool ViSensorConfiguration::emitCameraCalibration(
    YAML::Node* node, const ViCameraLensModelEquidistant::Ptr lens_model)
{

  (*node)[static_cast<int>(ViCameraLensModelEquidistant::EquidistantsCoefficients::K1)] =
      lens_model->k1_;
  (*node)[static_cast<int>(ViCameraLensModelEquidistant::EquidistantsCoefficients::K2)] =
      lens_model->k2_;
  (*node)[static_cast<int>(ViCameraLensModelEquidistant::EquidistantsCoefficients::K3)] =
      lens_model->k3_;
  (*node)[static_cast<int>(ViCameraLensModelEquidistant::EquidistantsCoefficients::K4)] =
      lens_model->k4_;
  return true;
}

bool ViSensorConfiguration::emitCameraCalibration(
    YAML::Node* node, const ViCameraProjectionModel::Ptr projection_model)
{
  YAML::Node node_coefficients;
  switch (projection_model->type_) {
    case ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL:
      (*node)[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::TYPE)] =
          yaml_camera_calibration_projectionmodel_type_.at(ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL);
      emitCameraCalibration(
          &node_coefficients,
          std::static_pointer_cast<ViCameraProjectionModelOmnidirectional>(projection_model));
      break;
    case ViCameraProjectionModel::ProjectionModelTypes::PINHOLE:
      (*node)[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::TYPE)] =
          yaml_camera_calibration_projectionmodel_type_.at(ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
      emitCameraCalibration(
          &node_coefficients,
          std::static_pointer_cast<ViCameraProjectionModelPinhole>(projection_model));
      break;
    default:
      (*node)[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::TYPE)] =
          yaml_camera_calibration_projectionmodel_type_.at(ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
      emitCameraCalibration(
          &node_coefficients,
          std::static_pointer_cast<ViCameraProjectionModelPinhole>(projection_model));
      VISENSOR_DEBUG("Projection Model not known");
  }
  (*node)[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::COEFFICIENTS)] =
      node_coefficients;

  return true;
}

bool ViSensorConfiguration::emitCameraCalibration(
    YAML::Node* node, const ViCameraProjectionModelOmnidirectional::Ptr projection_model)
{
  (*node)[static_cast<int>(ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::FU)] =
      projection_model->focal_length_u_;
  (*node)[static_cast<int>(ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::FV)] =
      projection_model->focal_length_v_;
  (*node)[static_cast<int>(ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::PU)] =
      projection_model->principal_point_u_;
  (*node)[static_cast<int>(ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::PV)] =
      projection_model->principal_point_v_;
  (*node)[static_cast<int>(ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::XI)] =
      projection_model->mirror_xi_;
  return true;
}

bool ViSensorConfiguration::emitCameraCalibration(
    YAML::Node* node, const ViCameraProjectionModelPinhole::Ptr projection_model)
{
  (*node)[static_cast<int>(ViCameraProjectionModelPinhole::PinholeCoefficients::FU)] =
      projection_model->focal_length_u_;
  (*node)[static_cast<int>(ViCameraProjectionModelPinhole::PinholeCoefficients::FV)] =
      projection_model->focal_length_v_;
  (*node)[static_cast<int>(ViCameraProjectionModelPinhole::PinholeCoefficients::PU)] =
      projection_model->principal_point_u_;
  (*node)[static_cast<int>(ViCameraProjectionModelPinhole::PinholeCoefficients::PV)] =
      projection_model->principal_point_v_;
  return true;
}

std::vector<ViCameraCalibration> ViSensorConfiguration::parseYaml(const YAML::Node& config_nodes)
{
  std::vector<ViCameraCalibration> camera_calibration_vector;
  if (!config_nodes[yaml_naming_.at(ConfigYaml_e::CALIBRATION)]) {
    VISENSOR_DEBUG("could not find camera calibrations\n");
    return camera_calibration_vector;
  }
  YAML::Node calibration_node = config_nodes[yaml_naming_.at(ConfigYaml_e::CALIBRATION)];

  int i = 0;
  // iteration over all camera calibrations
  for (YAML::const_iterator cam_it = calibration_node.begin(); cam_it != calibration_node.end();
      ++cam_it, i++) {
    // check if not empty
    YAML::Node cam_node = (*cam_it)[yaml_camera_.at(ConfigYamlCamera_e::CAMERA)];
    if (cam_node.IsMap()) {

      int camera_number = cam_node[yaml_camera_.at(ConfigYamlCamera_e::CAM_NUMBER)].as<int>();

      try {
        YAML::Node calibration_node = cam_node[yaml_camera_.at(ConfigYamlCamera_e::CALIBRATION)];

        if (calibration_node.Type() == YAML::NodeType::Map) {
          ViCameraCalibration camera_calibration;
          parseYaml(calibration_node, &camera_calibration, camera_number);
          camera_calibration_vector.push_back(camera_calibration);
        } else if (calibration_node.Type() == YAML::NodeType::Sequence) {
          for (const YAML::Node& calibration_entries : calibration_node) {
            if (calibration_entries.Type() == YAML::NodeType::Map) {
              ViCameraCalibration camera_calibration;
              parseYaml(calibration_entries, &camera_calibration, camera_number);
              camera_calibration_vector.push_back(camera_calibration);
            } else
              VISENSOR_DEBUG("calibration values is not a map or sequence\n");
          }
        } else
          VISENSOR_DEBUG("calibration values is not a map or sequence\n");
      } catch (visensor::exceptions const &ex) {
        VISENSOR_DEBUG("could not parse the calibration correctly, %s\n", ex.what());
      } catch (YAML::Exception const &ex) {
        VISENSOR_DEBUG("could not parse the calibration correctly: %s\n", ex.what());
      }
    }
  }
  return camera_calibration_vector;
}

// now the extraction operators for these types
void ViSensorConfiguration::parseYaml(const YAML::Node& node,
                                      ViCameraProjectionModelPinhole::Ptr projection_model)
{
  for (unsigned short i = 0; i < node.size(); ++i) {
    switch (static_cast<ViCameraProjectionModelPinhole::PinholeCoefficients>(i)) {
      case ViCameraProjectionModelPinhole::PinholeCoefficients::FU:
        projection_model->focal_length_u_ = node[i].as<double>();
        break;
      case ViCameraProjectionModelPinhole::PinholeCoefficients::FV:
        projection_model->focal_length_v_ = node[i].as<double>();
        break;
      case ViCameraProjectionModelPinhole::PinholeCoefficients::PU:
        projection_model->principal_point_u_ = node[i].as<double>();
        break;
      case ViCameraProjectionModelPinhole::PinholeCoefficients::PV:
        projection_model->principal_point_v_ = node[i].as<double>();
        break;
      default:
        throw visensor::exceptions::ConfigException(
            "could not parse ViCameraProjectionModelPinhole coefficient, size mismatch:  "
                + node.size());
    }
  }
}

void ViSensorConfiguration::parseYaml(const YAML::Node& node,
                                      ViCameraProjectionModelOmnidirectional::Ptr projection_model)
{
  for (unsigned short i = 0; i < node.size(); ++i) {
    switch (static_cast<ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients>(i)) {
      case ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::FU:
        projection_model->focal_length_u_ = node[i].as<double>();
        break;
      case ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::FV:
        projection_model->focal_length_v_ = node[i].as<double>();
        break;
      case ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::PU:
        projection_model->principal_point_u_ = node[i].as<double>();
        break;
      case ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::PV:
        projection_model->principal_point_v_ = node[i].as<double>();
        break;
      case ViCameraProjectionModelOmnidirectional::OmnidirectionalCoefficients::XI:
        projection_model->mirror_xi_ = node[i].as<double>();
        break;
      default:
        throw visensor::exceptions::ConfigException(
            "could not parse ViCameraProjectionModelOmnidirectional coefficient, size mismatch:  "
                + node.size());
    }
  }
}

void ViSensorConfiguration::parseYaml(const YAML::Node& node,
                                      ViCameraLensModelEquidistant::Ptr lens_model)
{
  for (unsigned short i = 0; i < node.size(); ++i) {
    switch (static_cast<ViCameraLensModelEquidistant::EquidistantsCoefficients>(i)) {
      case ViCameraLensModelEquidistant::EquidistantsCoefficients::K1:
        lens_model->k1_ = node[i].as<double>();
        break;
      case ViCameraLensModelEquidistant::EquidistantsCoefficients::K2:
        lens_model->k2_ = node[i].as<double>();
        break;
      case ViCameraLensModelEquidistant::EquidistantsCoefficients::K3:
        lens_model->k3_ = node[i].as<double>();
        break;
      case ViCameraLensModelEquidistant::EquidistantsCoefficients::K4:
        lens_model->k4_ = node[i].as<double>();
        break;
      default:
        throw visensor::exceptions::ConfigException(
            "could not parse ViCameraLensModelEquidistant coefficient, size mismatch:  "
                + node.size());
    }
  }
}

void ViSensorConfiguration::parseYaml(const YAML::Node& node,
                                      ViCameraLensModelRadtan::Ptr lens_model)
{
  for (unsigned short i = 0; i < node.size(); ++i) {
    switch (static_cast<ViCameraLensModelRadtan::RadtanCoefficients>(i)) {
      case ViCameraLensModelRadtan::RadtanCoefficients::K1:
        lens_model->k1_ = node[i].as<double>();
        break;
      case ViCameraLensModelRadtan::RadtanCoefficients::K2:
        lens_model->k2_ = node[i].as<double>();
        break;
      case ViCameraLensModelRadtan::RadtanCoefficients::R1:
        lens_model->r1_ = node[i].as<double>();
        break;
      case ViCameraLensModelRadtan::RadtanCoefficients::R2:
        lens_model->r2_ = node[i].as<double>();
        break;
      default:
        throw visensor::exceptions::ConfigException(
            "could not parse ViCameraLensModelRadTan coefficient, size mismatch:  " + node.size());
    }
  }
}

void ViSensorConfiguration::parseYaml(const YAML::Node& node, ViCameraCalibration* calibration,
                                      int camera_number)
{
  if (node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::LENS_MODEL)]) {
    const YAML::Node& lens_model_node =
        node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::LENS_MODEL)];
    for (auto it = yaml_camera_calibration_lensmodel_type_.begin();
        it != yaml_camera_calibration_lensmodel_type_.end(); ++it) {
      YAML::Node node_tmp =
          lens_model_node[yaml_camera_calibration_lensmodel_.at(ConfigYamlLensModelStruct_e::TYPE)];
      if (node_tmp.as<std::string>() == it->second) {
        switch (static_cast<ViCameraLensModel::LensModelTypes>(it->first)) {
          case ViCameraLensModel::LensModelTypes::EQUIDISTANT:
            calibration->lens_model_ = std::make_shared<visensor::ViCameraLensModelEquidistant>();
            parseYaml(
                lens_model_node[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::COEFFICIENTS)],
                std::static_pointer_cast<ViCameraLensModelEquidistant>(calibration->lens_model_));
            break;
          case ViCameraLensModel::LensModelTypes::RADTAN:
            calibration->lens_model_ = std::make_shared<visensor::ViCameraLensModelRadtan>();
            parseYaml(
                lens_model_node[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::COEFFICIENTS)],
                std::static_pointer_cast<ViCameraLensModelRadtan>(calibration->lens_model_));
            break;
          default:
            calibration->lens_model_ = std::make_shared<visensor::ViCameraLensModelRadtan>();
            parseYaml(
                lens_model_node[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::COEFFICIENTS)],
                std::static_pointer_cast<ViCameraLensModelRadtan>(calibration->lens_model_));
            VISENSOR_DEBUG("Lens model not known");
        }
        break;
      }
    }
  } else
    throw visensor::exceptions::ConfigException(
        "no node " + yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::LENS_MODEL)
            + " found");

  if (node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::PROJECTION_MODEL)]) {
    const YAML::Node& projection_model_node =
        node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::PROJECTION_MODEL)];
    for (auto it = yaml_camera_calibration_projectionmodel_type_.begin();
        it != yaml_camera_calibration_projectionmodel_type_.end(); ++it) {
      YAML::Node node_tmp =
          projection_model_node[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::TYPE)];
      if (node_tmp.as<std::string>() == it->second) {
        switch (static_cast<ViCameraProjectionModel::ProjectionModelTypes>(it->first)) {
          case ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL:
            calibration->projection_model_ = std::make_shared<
                visensor::ViCameraProjectionModelOmnidirectional>();
            parseYaml(
                projection_model_node[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::COEFFICIENTS)],
                std::static_pointer_cast<ViCameraProjectionModelOmnidirectional>(
                    calibration->projection_model_));
            break;
          case ViCameraProjectionModel::ProjectionModelTypes::PINHOLE:
            calibration->projection_model_ = std::make_shared<
                visensor::ViCameraProjectionModelPinhole>();
            parseYaml(
                projection_model_node[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::COEFFICIENTS)],
                std::static_pointer_cast<ViCameraProjectionModelPinhole>(
                    calibration->projection_model_));
            break;
          default:
            calibration->projection_model_ = std::make_shared<
                visensor::ViCameraProjectionModelPinhole>();
            parseYaml(
                projection_model_node[yaml_camera_calibration_projectionmodel_.at(ConfigYamlCameraProjectionModelStruct_e::COEFFICIENTS)],
                std::static_pointer_cast<ViCameraProjectionModelPinhole>(
                    calibration->projection_model_));
            VISENSOR_DEBUG("Projection Model not known");
        }
        break;
      }
    }
  } else
    throw visensor::exceptions::ConfigException(
        "no node " + yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::PROJECTION_MODEL)
            + " found");

  if (node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::R)].size() != 9)
    throw visensor::exceptions::ConfigException(
        "could not parse R parameter, size mismatch:  "
            + node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::R)].size());
  for (unsigned short i = 0;
      i < node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::R)].size(); ++i) {
    calibration->R_.push_back(
        node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::R)][i].as<double>());
  }

  if (node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::T)].size() != 3)
    throw visensor::exceptions::ConfigException(
        "could not parse t parameter, size mismatch:  "
            + node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::T)].size());
  for (unsigned short i = 0;
      i < node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::T)].size(); ++i) {
    calibration->t_.push_back(
        node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::T)][i].as<double>());
  }

  if (node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::RESOLUTION)].size() != 2)
    visensor::exceptions::ConfigException(
        "could not parse resolution parameter, size mismatch:  "
            + node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::RESOLUTION)].size());
  for (unsigned short i = 0; i < 2; i++) {
    calibration->resolution_[i] =
        node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::RESOLUTION)][i].as<int>();
  }

  if (node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::SLOT_ID)])
    calibration->slot_id_ =
        node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::SLOT_ID)].as<int>();
  else
    throw visensor::exceptions::ConfigException(
        "no node " + yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::SLOT_ID) + " found");

  if (node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::IS_FLIPPED)])
    calibration->is_flipped_ =
        node[yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::IS_FLIPPED)].as<bool>();
  else
    throw visensor::exceptions::ConfigException(
        "no node " + yaml_camera_calibration_.at(ConfigYamlCameraCalibration_e::IS_FLIPPED)
            + " found");

  calibration->cam_id_ = camera_number;
}

bool ViSensorConfiguration::isValid() const
{
  return valid_;
}
}

