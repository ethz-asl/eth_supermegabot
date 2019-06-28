/*!
* @file    AverageForceCalibratorTest.cpp
* @author  Christian Gehring
* @date    Apr, 2016
*/

#include <gtest/gtest.h>
#include "robot_utils/force_calibrators/AverageForceCalibrator.hpp"

volatile static int g_run = 1;

void run(robot_utils::AverageForceCalibrator* calibration) {
//  std::cout << "start thread" << std::endl;
  int sampleCounts = 0;

  while (g_run) {
    kindr::Force3D force;

    if (calibration->isCalibrating()) {

//      printf("run: is calibrating\n");

      if (sampleCounts == 0) {
        force = kindr::Force3D(2.0, 2.0, 2.0);
      }
      else if (sampleCounts == 1) {
        force = kindr::Force3D(4.0, 4.0, 4.0);
      }
        sampleCounts++;
//      std::cout << "sample force " << force.transpose() << std::endl;
    }


//    std::cout <<  "run: update with force: " << force.transpose() << std::endl;
    calibration->setForce(force);
    calibration->advance(0.01);
    sleep(1);
  }
}

TEST(AverageForceCalibratorTest, DISABLED_calibrate)
{
  using namespace robot_utils;
  AverageForceCalibrator::Config config;
  config.numSamples_ = 2;
  config.minNumGoodSamples_ = 2;
  AverageForceCalibrator calibration("lf_foot", config);
  boost::thread thread{run, &calibration};
  sleep(2);
  calibration.startCalibration(true);
  g_run = 0;

  kindr::Force3D calibratedForce;
  ASSERT_TRUE(calibration.getCalibratedForce(calibratedForce, kindr::Force3D(0.0, 0.0, 0.0)));
  EXPECT_EQ(-3.0, calibratedForce.x());
  EXPECT_EQ(-3.0, calibratedForce.y());
  EXPECT_EQ(-3.0, calibratedForce.z());

//  Eigen::Vector3d offset = calibration.getForceOffset();
//  EXPECT_EQ(3.0, offset.x());
//  EXPECT_EQ(3.0, offset.y());
//  EXPECT_EQ(3.0, offset.z());
////  std::cout << calibration.getForceOffset() << std::endl;
//  Eigen::Vector3d  calibratedforce = calibration.update( Eigen::Vector3d(6.0, 6.0, 6.0));
//  EXPECT_EQ(3.0, calibratedforce.x());
//  EXPECT_EQ(3.0, calibratedforce.y());
//  EXPECT_EQ(3.0, calibratedforce.z());
}

TEST(AverageForceCalibratorTest, calibrateSimple)
{
  using namespace robot_utils;
  double timeStep = 0.01;
  AverageForceCalibrator::Config config;
  config.numSamples_ = 2;
  config.minNumGoodSamples_ = 2;
  config.calibrateAfterSampling_ = true;
  config.alphaFilterForce_ = 1.0;
  AverageForceCalibrator calibrator("lf_foot", config);
  ASSERT_FALSE(calibrator.isCalibrating());
  ASSERT_TRUE(calibrator.startCalibration(false));
  calibrator.setForce(kindr::Force3D(1.0, 0.1, 10.0));
  calibrator.advance(timeStep);
  ASSERT_TRUE(calibrator.isCalibrating());
  calibrator.setForce(kindr::Force3D(2.0, 0.2, 20.0));
  calibrator.advance(timeStep);
  ASSERT_FALSE(calibrator.isCalibrating());
  kindr::WrenchD wrenchOffset;

  // additional sample that should not be considered
  calibrator.setForce(kindr::Force3D(5.0, 0.5, 50.0));
  calibrator.advance(timeStep);

  calibrator.getWrenchOffset(wrenchOffset);
  EXPECT_NEAR(1.5, wrenchOffset.getForce().x(), 1.0e-6);
  EXPECT_NEAR(0.15, wrenchOffset.getForce().y(), 1.0e-6);
  EXPECT_NEAR(15.0, wrenchOffset.getForce().z(), 1.0e-6);

  kindr::Force3D calibratedForce;
  ASSERT_TRUE(calibrator.getCalibratedForce(calibratedForce, kindr::Force3D(0.0, 0.0, 0.0)));
  EXPECT_NEAR(-1.5, calibratedForce.x(), 1.0e-6);
  EXPECT_NEAR(-0.15, calibratedForce.y(), 1.0e-6);
  EXPECT_NEAR(-15.0, calibratedForce.z(), 1.0e-6);



  // Check continue sampling though it is not calibrating
  ASSERT_FALSE(calibrator.continueSampling());
  ASSERT_FALSE(calibrator.isCalibrating());

  // Set new config, this time without stopping the calibration
  config.calibrateAfterSampling_ = false;
  config.numSamples_ = 1;
  config.minNumGoodSamples_ = 1;
  calibrator.setConfig(config);

  // Start calibration with new config
  ASSERT_TRUE(calibrator.startCalibration(false));
  ASSERT_TRUE(calibrator.isCalibrating());
  calibrator.setForce(kindr::Force3D(7.0, 0.7, 70.0));
  calibrator.advance(timeStep);

  // Should not stop calibration because config.calibrateAfterSampling_ = false;
  ASSERT_TRUE(calibrator.isCalibrating());
  wrenchOffset.setZero();
  calibrator.getWrenchOffset(wrenchOffset);
  EXPECT_NEAR(1.5, wrenchOffset.getForce().x(), 1.0e-6);
  EXPECT_NEAR(0.15, wrenchOffset.getForce().y(), 1.0e-6);
  EXPECT_NEAR(15.0, wrenchOffset.getForce().z(), 1.0e-6);
  calibrator.stopCalibration();
  wrenchOffset.setZero();
  calibrator.getWrenchOffset(wrenchOffset);
  ASSERT_FALSE(calibrator.isCalibrating());
  EXPECT_NEAR(7.0, wrenchOffset.getForce().x(), 1.0e-6);
  EXPECT_NEAR(0.7, wrenchOffset.getForce().y(), 1.0e-6);
  EXPECT_NEAR(70.0, wrenchOffset.getForce().z(), 1.0e-6);


  // Set new config, this time without stopping the calibration
  config.calibrateAfterSampling_ = false;
  config.numSamples_ = 2;
  config.minNumGoodSamples_ = 1;
  calibrator.setConfig(config);
  ASSERT_TRUE(calibrator.startCalibration(false));
  calibrator.setForce(kindr::Force3D(9.0, 0.9, 90.0));
  calibrator.advance(timeStep);
  calibrator.stopCalibration();
  wrenchOffset.setZero();
  calibrator.getWrenchOffset(wrenchOffset);
  ASSERT_FALSE(calibrator.isCalibrating());
  EXPECT_NEAR(9.0, wrenchOffset.getForce().x(), 1.0e-6);
  EXPECT_NEAR(0.9, wrenchOffset.getForce().y(), 1.0e-6);
  EXPECT_NEAR(90.0, wrenchOffset.getForce().z(), 1.0e-6);


  config.calibrateAfterSampling_ = false;
  config.numSamples_ = 1;
  config.minNumGoodSamples_ = 1;
  calibrator.setConfig(config);
  ASSERT_TRUE(calibrator.startCalibration(false));
  calibrator.setForce(kindr::Force3D(9.0, 0.9, 90.0));
  calibrator.advance(timeStep);
  calibrator.advance(timeStep); // should not update
  ASSERT_TRUE(calibrator.isCalibrating());
  ASSERT_TRUE(calibrator.continueSampling());
  calibrator.setForce(kindr::Force3D(0.0, 0.0, 0.0));
  calibrator.advance(timeStep);
  calibrator.stopCalibration();
  wrenchOffset.setZero();
  calibrator.getWrenchOffset(wrenchOffset);
  ASSERT_FALSE(calibrator.isCalibrating());
  EXPECT_NEAR(4.5, wrenchOffset.getForce().x(), 1.0e-6);
  EXPECT_NEAR(0.45, wrenchOffset.getForce().y(), 1.0e-6);
  EXPECT_NEAR(45.0, wrenchOffset.getForce().z(), 1.0e-6);
}

TEST(AverageForceCalibratorTest, command)
{
  using namespace robot_utils;
  double timeStep = 0.01;
  kindr::WrenchD wrenchOffset;
  AverageForceCalibrator::Config config;
  config.numSamples_ = 10;
  config.minNumGoodSamples_ = 10;
  config.calibrateAfterSampling_ = true;
  config.alphaFilterForce_ = 1.0;
  AverageForceCalibrator calibrator("lf_foot", config);
  ForceCalibratorCommand command;
  command.cmdCalibrate_ = true;
  command.cmdStart_ = true;
  command.cmdContinue_ = false;
  command.numSamples_ = 2;
  command.numGoodSamples_ = 2;
  calibrator.command(command);
  ASSERT_FALSE(command.cmdStart_);
  ASSERT_TRUE(calibrator.isCalibrating());
  calibrator.setForce(kindr::Force3D(1.0, 0.1, 10.0));
  calibrator.advance(timeStep);
  ASSERT_TRUE(calibrator.isCalibrating());
  calibrator.setForce(kindr::Force3D(2.0, 0.2, 20.0));
  calibrator.advance(timeStep);
  ASSERT_FALSE(calibrator.isCalibrating());
  wrenchOffset.setZero();
  calibrator.getWrenchOffset(wrenchOffset);
  EXPECT_NEAR(1.5, wrenchOffset.getForce().x(), 1.0e-6);
  EXPECT_NEAR(0.15, wrenchOffset.getForce().y(), 1.0e-6);
  EXPECT_NEAR(15.0, wrenchOffset.getForce().z(), 1.0e-6);

  command.numSamples_ = 1;
  command.numGoodSamples_ = 1;
  command.cmdCalibrate_ = false;
  command.cmdStart_ = true;
  command.cmdContinue_ = false;
  calibrator.command(command);
  calibrator.setForce(kindr::Force3D(1.0, 0.1, 10.0));
  calibrator.advance(timeStep);
  ASSERT_TRUE(calibrator.isCalibrating());
  calibrator.advance(timeStep); // should not do anything
  command.cmdCalibrate_ = true;
  command.cmdContinue_ = true;
  calibrator.command(command);
  calibrator.setForce(kindr::Force3D(5.0, 0.5, 50.0));
  calibrator.advance(timeStep);
  ASSERT_FALSE(calibrator.isCalibrating());
  wrenchOffset.setZero();
  calibrator.getWrenchOffset(wrenchOffset);
  EXPECT_NEAR(3.0, wrenchOffset.getForce().x(), 1.0e-6);
  EXPECT_NEAR(0.3, wrenchOffset.getForce().y(), 1.0e-6);
  EXPECT_NEAR(30.0, wrenchOffset.getForce().z(), 1.0e-6);

}

