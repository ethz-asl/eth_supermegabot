#include <fstream>
#include <memory>
#include <sstream>
#include <string>

// Bring in gtest
#include <gtest/gtest.h>

#include "config/visensor_configuration.hpp"
#include "networking/file_transfer.hpp"
#include "visensor/visensor_datatypes.hpp"

class FileTransferLocal : public visensor::FileTransfer
{
 public:
  typedef boost::shared_ptr<FileTransferLocal> Ptr;

  FileTransferLocal(visensor::SshConnection::Ptr connection)
      : visensor::FileTransfer(connection)
  {

  }
  virtual ~FileTransferLocal()
  {

  }

  virtual void downloadFile(const std::string& local_path, const std::string& remote_path)
  {

  }
  virtual void uploadFile(const std::string& local_path, const std::string& remote_path,
  bool remount = false)
  {

  }

  virtual void readRemoteFile(const std::string& file_path, std::string* file_content)
  {
    std::ifstream file;
    file.open(file_path.c_str());

    std::stringstream strStream;
    strStream << file.rdbuf();  //read the file
    *file_content = strStream.str();  //str holds the content of the file
    file.close();
  }
  virtual void writeRemoteFile(const std::string& file_path, const std::string& file_content,
  bool remount = false)
  {
    std::ofstream file;

    file.open(file_path.c_str());
    file << file_content;
    file.close();
  }

 private:
};

class Visensor_configuration : public ::testing::Test
{
 protected:
  virtual void SetUp()
  {

    ssh_connection_ = boost::make_shared<visensor::SshConnection>();
    file_transfer_ = boost::make_shared<FileTransferLocal>(ssh_connection_);
    config_ = boost::make_shared<visensor::ViSensorConfiguration>(file_transfer_);
  }
  virtual void TearDown()
  {
    file_transfer_ = NULL;
    config_ = NULL;
    if (file_exists(TEST_OUTPUTFILE_PATH))
      std::remove(TEST_OUTPUTFILE_PATH.c_str());

  }

  inline bool file_exists(const std::string& name)
  {
    if (FILE *file = fopen(name.c_str(), "r")) {
      fclose(file);
      return true;
    } else {
      return false;
    }
  }

  const std::string TEST_FILE_PATH = "testConfig.yaml";
  const std::string TEST_OUTPUTFILE_PATH = "testfile.yaml";
  FileTransferLocal::Ptr file_transfer_;
  visensor::ViSensorConfiguration::Ptr config_;
  visensor::SshConnection::Ptr ssh_connection_;
};

TEST_F(Visensor_configuration, TestVisensorConfigurationInitConfigClass)
{
  const int CAM_ID = 1;
  const int SLOT_ID = 5;
  const bool IS_FLIPPED = 0;
  visensor::ViCameraCalibration calibration(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL, 100.1, 100.2, 100.3,
      100.4, 100.5, 100.6, 100.7, 100.8, 100.9);

  EXPECT_EQ(calibration.cam_id_, CAM_ID);
  EXPECT_EQ(calibration.slot_id_, SLOT_ID);
  EXPECT_EQ(calibration.is_flipped_, IS_FLIPPED);
  EXPECT_EQ(static_cast<long long int>(calibration.resolution_[0]), 1000);
  EXPECT_EQ(static_cast<long long int>(calibration.resolution_[1]), 200);
  EXPECT_EQ(calibration.lens_model_->type_,
            visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT);
  visensor::ViCameraLensModelEquidistant::Ptr lens_model = std::static_pointer_cast<
      visensor::ViCameraLensModelEquidistant>(calibration.lens_model_);
  EXPECT_EQ(lens_model->k1_, 100.1);
  EXPECT_EQ(lens_model->k2_, 100.2);
  EXPECT_EQ(lens_model->k3_, 100.3);
  EXPECT_EQ(lens_model->k4_, 100.4);
  EXPECT_EQ(calibration.projection_model_->type_,
            visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL);
  visensor::ViCameraProjectionModelOmnidirectional::Ptr projection_model = std::static_pointer_cast<
      visensor::ViCameraProjectionModelOmnidirectional>(calibration.projection_model_);
  EXPECT_EQ(projection_model->focal_length_u_, 100.5);
  EXPECT_EQ(projection_model->focal_length_v_, 100.6);
  EXPECT_EQ(projection_model->principal_point_u_, 100.7);
  EXPECT_EQ(projection_model->principal_point_v_, 100.8);
  EXPECT_EQ(projection_model->mirror_xi_, 100.9);

  visensor::ViCameraCalibration calibration2(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED, visensor::ViCameraLensModel::LensModelTypes::RADTAN,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE, 101.1, 101.2, 101.3, 101.4,
      101.5, 101.6, 101.7, 101.8);

  EXPECT_EQ(calibration2.cam_id_, CAM_ID);
  EXPECT_EQ(calibration2.slot_id_, SLOT_ID);
  EXPECT_EQ(calibration2.is_flipped_, IS_FLIPPED);
  EXPECT_EQ(static_cast<long long int>(calibration2.resolution_[0]), 1000);
  EXPECT_EQ(static_cast<long long int>(calibration2.resolution_[1]), 200);
  EXPECT_EQ(calibration2.lens_model_->type_, visensor::ViCameraLensModel::LensModelTypes::RADTAN);
  visensor::ViCameraLensModelEquidistant::Ptr lens_model2 = std::static_pointer_cast<
      visensor::ViCameraLensModelEquidistant>(calibration2.lens_model_);
  EXPECT_EQ(lens_model2->k1_, 101.1);
  EXPECT_EQ(lens_model2->k2_, 101.2);
  EXPECT_EQ(lens_model2->k3_, 101.3);
  EXPECT_EQ(lens_model2->k4_, 101.4);
  EXPECT_EQ(calibration2.projection_model_->type_,
            visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
  visensor::ViCameraProjectionModelPinhole::Ptr projection_model2 = std::static_pointer_cast<
      visensor::ViCameraProjectionModelPinhole>(calibration2.projection_model_);
  EXPECT_EQ(projection_model2->focal_length_u_, 101.5);
  EXPECT_EQ(projection_model2->focal_length_v_, 101.6);
  EXPECT_EQ(projection_model2->principal_point_u_, 101.7);
  EXPECT_EQ(projection_model2->principal_point_v_, 101.8);
}

TEST_F(Visensor_configuration, TestVisensorConfigurationWriteReadNew)
{
  const int CAM_ID = 1;
  const int SLOT_ID = 5;
  const bool IS_FLIPPED = 0;
  visensor::ViCameraCalibration calibration(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL, 100.1, 100.2, 100.3,
      100.4, 100.5, 100.6, 100.7, 100.8, 100.9);
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_R; i++) {
    calibration.R_.push_back(110 + i);
  }
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_T; i++) {
    calibration.t_.push_back(120 + i);
  }
  config_->setCameraCalibration(calibration);
  config_->saveConfig(TEST_OUTPUTFILE_PATH);
  config_->loadConfig(TEST_OUTPUTFILE_PATH);

  std::vector<visensor::ViCameraCalibration> calibration_back = config_->getCameraCalibrations(
      static_cast<visensor::SensorId::SensorId>(CAM_ID), SLOT_ID, IS_FLIPPED);
  ASSERT_EQ(calibration_back.size(), 1);

  visensor::ViCameraCalibration camera_config = *calibration_back.begin();
  EXPECT_EQ(camera_config.cam_id_, calibration.cam_id_);
  EXPECT_EQ(camera_config.slot_id_, calibration.slot_id_);
  EXPECT_EQ(camera_config.is_flipped_, calibration.is_flipped_);
  EXPECT_EQ(static_cast<long long int>(camera_config.resolution_[0]),
            static_cast<long long int>(calibration.resolution_[0]));
  EXPECT_EQ(static_cast<long long int>(camera_config.resolution_[1]),
            static_cast<long long int>(calibration.resolution_[1]));
  EXPECT_EQ(camera_config.projection_model_->type_,
            visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL);
  visensor::ViCameraProjectionModelOmnidirectional::Ptr projection_model = std::static_pointer_cast<
      visensor::ViCameraProjectionModelOmnidirectional>(camera_config.projection_model_);
  visensor::ViCameraProjectionModelOmnidirectional::Ptr projection_model_orig =
      std::static_pointer_cast<visensor::ViCameraProjectionModelOmnidirectional>(
          calibration.projection_model_);
  EXPECT_EQ(projection_model->focal_length_u_, projection_model_orig->focal_length_u_);
  EXPECT_EQ(projection_model->focal_length_v_, projection_model_orig->focal_length_v_);
  EXPECT_EQ(projection_model->principal_point_u_, projection_model_orig->principal_point_u_);
  EXPECT_EQ(projection_model->principal_point_v_, projection_model_orig->principal_point_v_);
  EXPECT_EQ(projection_model->mirror_xi_, projection_model_orig->mirror_xi_);
  EXPECT_EQ(camera_config.lens_model_->type_,
            visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT);
  visensor::ViCameraLensModelEquidistant::Ptr lens_model = std::static_pointer_cast<
      visensor::ViCameraLensModelEquidistant>(camera_config.lens_model_);
  visensor::ViCameraLensModelEquidistant::Ptr lens_model_orig = std::static_pointer_cast<
      visensor::ViCameraLensModelEquidistant>(calibration.lens_model_);
  EXPECT_EQ(lens_model->k1_, lens_model_orig->k1_);
  EXPECT_EQ(lens_model->k2_, lens_model_orig->k2_);
  EXPECT_EQ(lens_model->k3_, lens_model_orig->k3_);
  EXPECT_EQ(lens_model->k4_, lens_model_orig->k4_);

  for (double i = 0; i < 9; i++) {
    EXPECT_EQ(camera_config.R_[i], calibration.R_[i]);
  }
  for (uint i = 0; i < 3; i++) {
    EXPECT_EQ(camera_config.t_[i], calibration.t_[i]);
  }
}

TEST_F(Visensor_configuration, TestVisensorConfigurationReadConfigAndCheckOne)
{
  config_->loadConfig(TEST_FILE_PATH);
  EXPECT_EQ(config_->isValid(), true);
  std::vector<visensor::ViCameraCalibration> camera_configs = config_->getCameraCalibrations(
      static_cast<visensor::SensorId::SensorId>(0), 0, 0);

  ASSERT_EQ(camera_configs.size(), 1);

  visensor::ViCameraCalibration camera_config = *camera_configs.begin();
  EXPECT_EQ(camera_config.cam_id_, 0);
  EXPECT_EQ(camera_config.slot_id_, 0);
  EXPECT_EQ(camera_config.is_flipped_, 0);
  EXPECT_EQ(static_cast<long long int>(camera_config.resolution_[0]), 480);
  EXPECT_EQ(static_cast<long long int>(camera_config.resolution_[1]), 752);
  EXPECT_EQ(camera_config.projection_model_->type_,
            visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
  visensor::ViCameraProjectionModelPinhole::Ptr projection_model = std::static_pointer_cast<
      visensor::ViCameraProjectionModelPinhole>(camera_config.projection_model_);
  EXPECT_EQ(projection_model->focal_length_u_, 0.1);
  EXPECT_EQ(projection_model->focal_length_v_, 1.1);
  EXPECT_EQ(projection_model->principal_point_u_, 2.1);
  EXPECT_EQ(projection_model->principal_point_v_, 3.1);

  EXPECT_EQ(camera_config.lens_model_->type_, visensor::ViCameraLensModel::LensModelTypes::RADTAN);
  visensor::ViCameraLensModelRadtan::Ptr lens_model = std::static_pointer_cast<
      visensor::ViCameraLensModelRadtan>(camera_config.lens_model_);
  EXPECT_EQ(lens_model->k1_, 0.2);
  EXPECT_EQ(lens_model->k2_, 1.2);
  EXPECT_EQ(lens_model->r1_, 2.2);
  EXPECT_EQ(lens_model->r2_, 3.2);

  for (double i = 0; i < 9; i++) {
    EXPECT_EQ(camera_config.R_[i], (i + 1) / 10);
  }
  for (uint i = 0; i < 3; i++) {
    EXPECT_EQ(camera_config.t_[i], (i + 1) * 10);
  }
}

TEST_F(Visensor_configuration, TestVisensorConfigurationReadAndChange)
{
  const int CAM_ID = 1;
  const int SLOT_ID = 0;
  const bool IS_FLIPPED = 0;
  visensor::ViCameraCalibration calibration(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED, visensor::ViCameraLensModel::LensModelTypes::RADTAN,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE, 100.1, 100.2, 100.3, 100.4,
      100.5, 100.6, 100.7, 100.8);
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_R; i++) {
    calibration.R_.push_back(110 + i);
  }
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_T; i++) {
    calibration.t_.push_back(120 + i);
  }

  config_->loadConfig(TEST_FILE_PATH);
  config_->setCameraCalibration(calibration);
  config_->saveConfig(TEST_OUTPUTFILE_PATH);
  config_->loadConfig(TEST_OUTPUTFILE_PATH);

  std::vector<visensor::ViCameraCalibration> calibration_back = config_->getCameraCalibrations(
      static_cast<visensor::SensorId::SensorId>(CAM_ID), SLOT_ID, IS_FLIPPED);
  ASSERT_EQ(calibration_back.size(), 1);

  visensor::ViCameraCalibration camera_config = *calibration_back.begin();
  EXPECT_EQ(camera_config.cam_id_, calibration.cam_id_);
  EXPECT_EQ(camera_config.slot_id_, calibration.slot_id_);
  EXPECT_EQ(camera_config.is_flipped_, calibration.is_flipped_);
  EXPECT_EQ(static_cast<long long int>(camera_config.resolution_[0]),
            static_cast<long long int>(calibration.resolution_[0]));
  EXPECT_EQ(static_cast<long long int>(camera_config.resolution_[1]),
            static_cast<long long int>(calibration.resolution_[1]));
  EXPECT_EQ(camera_config.lens_model_->type_, visensor::ViCameraLensModel::LensModelTypes::RADTAN);
  visensor::ViCameraLensModelEquidistant::Ptr lens_model = std::static_pointer_cast<
      visensor::ViCameraLensModelEquidistant>(camera_config.lens_model_);
  visensor::ViCameraLensModelEquidistant::Ptr lens_model_orig = std::static_pointer_cast<
      visensor::ViCameraLensModelEquidistant>(calibration.lens_model_);
  EXPECT_EQ(lens_model->k1_, lens_model_orig->k1_);
  EXPECT_EQ(lens_model->k2_, lens_model_orig->k2_);
  EXPECT_EQ(lens_model->k3_, lens_model_orig->k3_);
  EXPECT_EQ(lens_model->k4_, lens_model_orig->k4_);
  EXPECT_EQ(camera_config.projection_model_->type_,
            visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
  visensor::ViCameraProjectionModelPinhole::Ptr projection_model = std::static_pointer_cast<
      visensor::ViCameraProjectionModelPinhole>(camera_config.projection_model_);
  visensor::ViCameraProjectionModelPinhole::Ptr projection_model_orig = std::static_pointer_cast<
      visensor::ViCameraProjectionModelPinhole>(calibration.projection_model_);
  EXPECT_EQ(projection_model->focal_length_u_, projection_model_orig->focal_length_u_);
  EXPECT_EQ(projection_model->focal_length_v_, projection_model_orig->focal_length_v_);
  EXPECT_EQ(projection_model->principal_point_u_, projection_model_orig->principal_point_u_);
  EXPECT_EQ(projection_model->principal_point_v_, projection_model_orig->principal_point_v_);

  for (double i = 0; i < 9; i++) {
    EXPECT_EQ(camera_config.R_[i], calibration.R_[i]);
  }
  for (uint i = 0; i < 3; i++) {
    EXPECT_EQ(camera_config.t_[i], calibration.t_[i]);
  }
}
//
TEST_F(Visensor_configuration, TestVisensorConfigurationReadAndAddOne)
{
  const int CAM_ID = 1;
  const int SLOT_ID = 0;
  const bool IS_FLIPPED = 0;
  visensor::ViCameraCalibration calibration(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL, 100.1, 100.2, 100.3,
      100.4, 100.5, 100.6, 100.7, 100.8, 100.9);
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_R; i++) {
    calibration.R_.push_back(110 + i);
  }
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_T; i++) {
    calibration.t_.push_back(120 + i);
  }

  config_->loadConfig(TEST_FILE_PATH);
  config_->setCameraCalibration(calibration);
  config_->saveConfig(TEST_OUTPUTFILE_PATH);
  config_->loadConfig(TEST_OUTPUTFILE_PATH);

  std::vector<visensor::ViCameraCalibration> calibration_back = config_->getCameraCalibrations(
      static_cast<visensor::SensorId::SensorId>(CAM_ID), SLOT_ID, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL);
  ASSERT_EQ(calibration_back.size(), 1);

  visensor::ViCameraCalibration camera_config = *calibration_back.begin();
  EXPECT_EQ(camera_config.cam_id_, calibration.cam_id_);
  EXPECT_EQ(camera_config.slot_id_, calibration.slot_id_);
  EXPECT_EQ(camera_config.is_flipped_, calibration.is_flipped_);
  EXPECT_EQ(static_cast<long long int>(camera_config.resolution_[0]),
            static_cast<long long int>(calibration.resolution_[0]));
  EXPECT_EQ(static_cast<long long int>(camera_config.resolution_[1]),
            static_cast<long long int>(calibration.resolution_[1]));
  EXPECT_EQ(camera_config.projection_model_->type_,
            visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL);
  visensor::ViCameraProjectionModelOmnidirectional::Ptr projection_model = std::static_pointer_cast<
      visensor::ViCameraProjectionModelOmnidirectional>(camera_config.projection_model_);
  visensor::ViCameraProjectionModelOmnidirectional::Ptr projection_model_orig =
      std::static_pointer_cast<visensor::ViCameraProjectionModelOmnidirectional>(
          calibration.projection_model_);
  EXPECT_EQ(projection_model->focal_length_u_, projection_model_orig->focal_length_u_);
  EXPECT_EQ(projection_model->focal_length_v_, projection_model_orig->focal_length_v_);
  EXPECT_EQ(projection_model->principal_point_u_, projection_model_orig->principal_point_u_);
  EXPECT_EQ(projection_model->principal_point_v_, projection_model_orig->principal_point_v_);
  EXPECT_EQ(projection_model->mirror_xi_, projection_model_orig->mirror_xi_);
  EXPECT_EQ(camera_config.lens_model_->type_,
            visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT);
  visensor::ViCameraLensModelEquidistant::Ptr lens_model = std::static_pointer_cast<
      visensor::ViCameraLensModelEquidistant>(camera_config.lens_model_);
  visensor::ViCameraLensModelEquidistant::Ptr lens_model_orig = std::static_pointer_cast<
      visensor::ViCameraLensModelEquidistant>(calibration.lens_model_);
  EXPECT_EQ(lens_model->k1_, lens_model_orig->k1_);
  EXPECT_EQ(lens_model->k2_, lens_model_orig->k2_);
  EXPECT_EQ(lens_model->k3_, lens_model_orig->k3_);
  EXPECT_EQ(lens_model->k4_, lens_model_orig->k4_);

  for (double i = 0; i < 9; i++) {
    EXPECT_EQ(camera_config.R_[i], calibration.R_[i]);
  }
  for (uint i = 0; i < 3; i++) {
    EXPECT_EQ(camera_config.t_[i], calibration.t_[i]);
  }
}

TEST_F(Visensor_configuration, TestVisensorConfigurationReadBackNonMatchingSlot)
{
  const int CAM_ID = 5;
  const int SLOT_ID = 0;
  const bool IS_FLIPPED = 0;
  visensor::ViCameraCalibration calibration(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL, 100.1, 100.2, 100.3,
      100.4, 100.5, 100.6, 100.7, 100.8, 100.9);
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_R; i++) {
    calibration.R_.push_back(110 + i);
  }
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_T; i++) {
    calibration.t_.push_back(120 + i);
  }

  config_->loadConfig(TEST_FILE_PATH);
  config_->setCameraCalibration(calibration);
  config_->saveConfig(TEST_OUTPUTFILE_PATH);
  config_->loadConfig(TEST_OUTPUTFILE_PATH);

  std::vector<visensor::ViCameraCalibration> calibration_back = config_->getCameraCalibrations(
      static_cast<visensor::SensorId::SensorId>(CAM_ID), SLOT_ID + 1, IS_FLIPPED);
  EXPECT_EQ(calibration_back.size(), 0);
}

TEST_F(Visensor_configuration, TestVisensorConfigurationReadBackNonMatchingLensModel)
{
  const int CAM_ID = 5;
  const int SLOT_ID = 0;
  const bool IS_FLIPPED = 0;
  visensor::ViCameraCalibration calibration(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL, 100.1, 100.2, 100.3,
      100.4, 100.5, 100.6, 100.7, 100.8, 100.9);
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_R; i++) {
    calibration.R_.push_back(110 + i);
  }
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_T; i++) {
    calibration.t_.push_back(120 + i);
  }

  config_->loadConfig(TEST_FILE_PATH);
  config_->setCameraCalibration(calibration);
  config_->saveConfig(TEST_OUTPUTFILE_PATH);
  config_->loadConfig(TEST_OUTPUTFILE_PATH);

  std::vector<visensor::ViCameraCalibration> calibration_back = config_->getCameraCalibrations(
      static_cast<visensor::SensorId::SensorId>(CAM_ID), SLOT_ID, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::RADTAN,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL);
  EXPECT_EQ(calibration_back.size(), 0);
}

TEST_F(Visensor_configuration, TestVisensorConfigurationReadBackGetMultiple)
{
  const int CAM_ID = 5;
  const int SLOT_ID = 0;
  const bool IS_FLIPPED = 0;
  visensor::ViCameraCalibration calibration(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL, 100.1, 100.2, 100.3,
      100.4, 100.5, 100.6, 100.7, 100.8, 100.9);
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_R; i++) {
    calibration.R_.push_back(110 + i);
  }
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_T; i++) {
    calibration.t_.push_back(120 + i);
  }
  visensor::ViCameraCalibration calibration2(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED, visensor::ViCameraLensModel::LensModelTypes::RADTAN,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL, 100.1, 100.2, 100.3,
      100.4, 100.5, 100.6, 100.7, 100.8, 100.9);
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_R; i++) {
    calibration2.R_.push_back(110 + i);
  }
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_T; i++) {
    calibration2.t_.push_back(120 + i);
  }
  visensor::ViCameraCalibration calibration3(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE, 100.1, 100.2, 100.3, 100.4,
      100.5, 100.6, 100.7, 100.8);
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_R; i++) {
    calibration3.R_.push_back(110 + i);
  }
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_T; i++) {
    calibration3.t_.push_back(120 + i);
  }
  visensor::ViCameraCalibration calibration4(
      CAM_ID, SLOT_ID, 1000, 200, IS_FLIPPED, visensor::ViCameraLensModel::LensModelTypes::RADTAN,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE, 100.1, 100.2, 100.3, 100.4,
      100.5, 100.6, 100.7, 100.8);
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_R; i++) {
    calibration4.R_.push_back(110 + i);
  }
  for (uint i = 0; i < visensor::ViCameraCalibration::NUMBER_OF_T; i++) {
    calibration4.t_.push_back(120 + i);
  }

  config_->loadConfig(TEST_FILE_PATH);
  config_->setCameraCalibration(calibration);
  config_->setCameraCalibration(calibration2);
  config_->setCameraCalibration(calibration3);
  config_->setCameraCalibration(calibration4);
  config_->saveConfig(TEST_OUTPUTFILE_PATH);
  config_->loadConfig(TEST_OUTPUTFILE_PATH);

  std::vector<visensor::ViCameraCalibration> calibration_back = config_->getCameraCalibrations(
      static_cast<visensor::SensorId::SensorId>(CAM_ID), SLOT_ID, IS_FLIPPED);
  EXPECT_EQ(calibration_back.size(), 4);
  calibration_back = config_->getCameraCalibrations(
      static_cast<visensor::SensorId::SensorId>(CAM_ID), SLOT_ID, IS_FLIPPED,
      visensor::ViCameraLensModel::LensModelTypes::RADTAN,
      visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL);
  EXPECT_EQ(calibration_back.size(), 1);
}

TEST_F(Visensor_configuration, TestVisensorConfigurationReadUserValues)
{
  config_->loadConfig(TEST_FILE_PATH);
  EXPECT_EQ(config_->isValid(), true);
  std::string testvalue;
  config_->getUserConfiguration("test", &testvalue);
  EXPECT_EQ(testvalue, "bla");
  int testinteger;
  config_->getUserConfiguration("testInteger", &testinteger);
  EXPECT_EQ(testinteger, 123);
}

TEST_F(Visensor_configuration, TestVisensorConfigurationWriteUserValues)
{
  config_->loadConfig(TEST_FILE_PATH);
  EXPECT_EQ(config_->isValid(), true);
  std::string testSetValue = "This is a test";
  config_->setUserConfiguration("test", testSetValue);
  config_->setUserConfiguration("testInteger", 1000);

  config_->saveConfig(TEST_OUTPUTFILE_PATH);
  config_->loadConfig(TEST_OUTPUTFILE_PATH);

  std::string testvalue;
  int testinteger;
  config_->getUserConfiguration("test", &testvalue);
  config_->getUserConfiguration("testInteger", &testinteger);
  EXPECT_EQ(testvalue, testSetValue);
  EXPECT_EQ(testinteger, 1000);
}

TEST_F(Visensor_configuration, TestVisensorConfigurationWriteNewUserValues)
{
  config_->loadConfig(TEST_FILE_PATH);
  EXPECT_EQ(config_->isValid(), true);
  config_->setUserConfiguration("newTestInteger", -1);

  config_->saveConfig(TEST_OUTPUTFILE_PATH);
  config_->loadConfig(TEST_OUTPUTFILE_PATH);

  int testinteger;
  config_->getUserConfiguration("newTestInteger", &testinteger);
  EXPECT_EQ(testinteger, -1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  try {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }

}
