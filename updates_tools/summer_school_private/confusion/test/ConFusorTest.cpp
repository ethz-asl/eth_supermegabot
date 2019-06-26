/*
 * Copyright 2018 Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//#define COST_DEBUG

#include <deque>
#include <iostream>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>

#include <confusion/utilities/imu_utils.h>
#include <confusion/utilities/Pose.h>

#include "confusion/BatchFusor.h"
#include "confusion/ConFusor.h"
#include "confusion/Diagram.h"
#include "confusion/StaticParameterVector.h"

#include "confusion/modules/apriltag/apriltag_utils.h"
#include "confusion/SensorEnumDefinition.h"
#include "confusion/ImuState.h"
#include "confusion/models/TagMeas.h"
#include "include_test/io_utils.h"
#include "confusion/TagTrackerParameters.h"

using namespace confusion;

boost::property_tree::ptree propertyTree;
Eigen::Matrix<double, 3, 4> projMat;
TagTrackerParameters solverParameters;
AprilTagParameters aprilTagParameters;
Eigen::Vector2d gravity_rot;
confusion::Pose<double> T_c_i;

std::vector<double> saveStaticParameterValues(const StaticParameterVector &staticParameters) {
  std::vector<double> values;
  for (auto &&staticParameter : staticParameters) {
    for (std::size_t i = 0; i < staticParameter.second.size(); ++i) {
      values.push_back(staticParameter.first[i]);
    }
  }
  return values;
}

void runConFusor(std::deque<std::shared_ptr<ImuMeas>> &imuMeasVec,
                 std::vector<std::shared_ptr<TagMeas>> &tagMeasVec) {
  const int numThreads = propertyTree.get<int>("vimhe.numThreads");
  const int maxNumIterations = propertyTree.get<int>("vimhe.maxNumIterations");
  const int batchSize = propertyTree.get<int>("vimhe.batchSize");
  const bool dropEverySecondState = propertyTree.get<bool>("vimhe.dropEverySecondState");
  bool optimizeTci = propertyTree.get<bool>("vimhe.optimizeTci");

  std::vector<Eigen::MatrixXd> stateParamInitialWeightings(5);
  Eigen::MatrixXd param_weighting(3, 3);
  param_weighting.setIdentity();
  param_weighting /= solverParameters.twi_init_stddev;
  stateParamInitialWeightings[0] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= solverParameters.qwi_init_stddev;
  stateParamInitialWeightings[1] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= solverParameters.vwi_init_stddev;
  stateParamInitialWeightings[2] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= solverParameters.ba_init_stddev;
  stateParamInitialWeightings[3] = param_weighting;

  param_weighting.setIdentity();
  param_weighting /= solverParameters.bg_init_stddev;
  stateParamInitialWeightings[4] = param_weighting;

  std::map<std::string, std::shared_ptr<confusion::Pose<double>>> referenceFrameOffsets;

  ConFusor conFusor(std::make_shared<ImuState>());

  // Link the states to the reference frame maps for adding new frames on the fly
  auto firstState = std::dynamic_pointer_cast<ImuState>(conFusor.stateVector_.front());
  firstState->setTagReferenceFrameOffsets(&referenceFrameOffsets);

  conFusor.stateVector_.front()->setInitialStateWeighting(stateParamInitialWeightings);
  conFusor.setSolverOptions(numThreads, maxNumIterations);
  bool removedStateLastTime = false;

  if (optimizeTci) {
    Eigen::MatrixXd t_c_i_initialWeighting(3, 3);
    t_c_i_initialWeighting.setIdentity();
    t_c_i_initialWeighting /= aprilTagParameters.t_c_i_init_stddev;
    confusion::Parameter tciParam(T_c_i.trans.data(), 3, "t_c_i");
    tciParam.setInitialConstraintWeighting(t_c_i_initialWeighting);
    conFusor.addStaticParameter(tciParam);
    Eigen::MatrixXd q_c_i_initialWeighting(3, 3);
    q_c_i_initialWeighting.setIdentity();
    q_c_i_initialWeighting /= aprilTagParameters.q_c_i_init_stddev;
    confusion::Parameter qciParam(T_c_i.rot.coeffs().data(), 4, "q_c_i", false, std::make_shared<QuatParam>());
    qciParam.setInitialConstraintWeighting(q_c_i_initialWeighting);
    conFusor.addStaticParameter(qciParam);
  } else {
    conFusor.addStaticParameter(confusion::Parameter(T_c_i.trans.data(), 3, "t_c_i", true));
    conFusor.addStaticParameter(confusion::Parameter(T_c_i.rot.coeffs().data(),
                                                     4,
                                                     "q_c_i",
                                                     true,
                                                     std::make_shared<QuatParam>()));
  }

  gravity_rot.setZero();

#ifdef OPT_GRAVITY
  Eigen::MatrixXd gravityRotInitialWeighting(2,2);
  gravityRotInitialWeighting.setIdentity();
  gravityRotInitialWeighting /= solverParameters.tagMeasCalibration_.ftr_init_stddev_;
  confusion::Parameter grParam(gravity_rot.data(), 2, "g_r");
  grParam.setInitialConstraintWeighting(gravityRotInitialWeighting);
  conFusor.addStaticParameter(grParam);
#endif

  ros::Time t_start = ros::Time::now();

  StateVector statesOut(NUM_PROCESS_SENSORS, NUM_UPDATE_SENSORS);
  //Step through the pose measurements, adding imu measurements and solving along the way
  size_t poseIndex = 0;
  size_t numPosesProcessed = 0;
  size_t imuIndex = 0;
  double t0 = tagMeasVec.front()->t();

  //Throw out old imu measurements
  while (imuMeasVec[imuIndex + 1]->t() <= tagMeasVec[poseIndex]->t()) {
    ++imuIndex;
  }

  std::vector<double> cycleTimes(tagMeasVec.size());
  int cycleTimeIndex = 0;
  while (poseIndex < tagMeasVec.size()) {
    ros::Time t_iter_start = ros::Time::now();

    while (imuIndex < imuMeasVec.size() &&
        imuMeasVec[imuIndex]->t() <= tagMeasVec[poseIndex]->t() + 0.01) {
      conFusor.addProcessMeasurement(imuMeasVec[imuIndex]);
      ++imuIndex;
    }

    double t_meas = tagMeasVec[poseIndex]->t();

    conFusor.addUpdateMeasurement(tagMeasVec[poseIndex]);
    ++poseIndex;

    //Add all pose measurements from the same image at the same time
    if (poseIndex < tagMeasVec.size() && tagMeasVec[poseIndex]->t() == t_meas) {
      continue;
    }

    if (!conFusor.assignMeasurements())
      continue; //No new states or measurements added, so wait for more measurements

    if (dropEverySecondState && conFusor.numStates() > batchSize - 2) {
      //Drop every second state for testing
      if (removedStateLastTime) {
        removedStateLastTime = false;
      } else {
        if (conFusor.numStates() > 2) {
          //Remove the second to last state
          conFusor.removeIntermediateState(conFusor.numStates() - 2);
          removedStateLastTime = true;
        }
      }
    }

    if (conFusor.numStates() > batchSize)
      conFusor.marginalizeFrontStates(1, &statesOut);

    conFusor.optimize();
//		conFusor.briefPrint();
    ros::Time t_iter_end = ros::Time::now();
//		std::cout << "con-fusion iteration took " << t_iter_end-t_iter_start << " sec.\n" << std::endl;

    ++numPosesProcessed;

    if (numPosesProcessed == 20) {
      std::string path = ros::package::getPath("confusion");
      std::string diagramFilename = path + "/data/ConfusorDiagram.png";
      Diagram diagram(conFusor, diagramFilename);
    }

    cycleTimes[cycleTimeIndex] = (t_iter_end - t_iter_start).toSec();
    ++cycleTimeIndex;
  }

  conFusor.stateVector_.back()->print();

  //Check that the tag poses make sense
  for (auto &referenceFrameOffset: referenceFrameOffsets) {
    EXPECT_LT(fabs(referenceFrameOffset.second->trans.norm()), 2.0);
    referenceFrameOffset.second->print("T_w_t");
  }

  double avgCycleTime = 0.0;
  double maxCycleTime = 0.0;
  for (int i = 0; i < cycleTimeIndex; ++i) {
    avgCycleTime += cycleTimes[i];
    if (cycleTimes[i] > maxCycleTime)
      maxCycleTime = cycleTimes[i];
  }
  avgCycleTime /= (double) cycleTimeIndex;
  std::cout << "\n\nConFusor avg cycle time: " << avgCycleTime << "; max cycle time: " << maxCycleTime << std::endl;

  for (int i = 0; i < conFusor.stateVector_.size(); ++i)
    statesOut.pushState(conFusor.stateVector_[i]);
  EXPECT_EQ(statesOut.size(), 92);

  //Optimize Tci
  conFusor.staticParameters_.getParameter(T_c_i.trans.data())->unsetConstant();
  conFusor.staticParameters_.getParameter(T_c_i.rot.coeffs().data())->unsetConstant();

  std::vector<double> conFusorStaticParameters = saveStaticParameterValues(conFusor.staticParameters_);

  // Disturb parameter to make it harder for batchFusor
  for (auto &&staticParameter : conFusor.staticParameters_) {
    for (std::size_t i = 0; i < staticParameter.second.size(); ++i) {
      staticParameter.first[i] * 0.9;
    }
  }

  BatchFusor batchFusor(statesOut, conFusor.staticParameters_);
  batchFusor.buildProblem();
  batchFusor.optimize();

  statesOut.back()->print();
  T_c_i.print("T_c_i");

  std::vector<double> batchFusorStaticParameters = saveStaticParameterValues(batchFusor.staticParameters_);

  EXPECT_EQ(conFusorStaticParameters.size(), batchFusorStaticParameters.size());
  // Test static parameters are similar for ConFusor and BatchFusor
  for (std::size_t i = 0; i < conFusorStaticParameters.size(); ++i) {
    EXPECT_NEAR(conFusorStaticParameters[i], batchFusorStaticParameters[i], 0.1);
  }

  //Check that tag poses make sense and quaternion norms
  for (auto &referenceFrameOffset: referenceFrameOffsets) {
    EXPECT_LT(fabs(referenceFrameOffset.second->trans.norm()), 2.0);
    referenceFrameOffset.second->print("T_w_t after batchsolve");

    EXPECT_LT(fabs(referenceFrameOffset.second->rot.norm() - 1.0), 1e-6);
  }
  auto testState = std::dynamic_pointer_cast<ImuState>(statesOut[5]);
  EXPECT_LT(fabs(testState->T_w_i_.rot.norm() - 1.0), 1e-6);
}

TEST(ConFusorTest, ShouldPass) {
  ros::Time::init();

  //Set the io file paths
  std::string path = ros::package::getPath("confusion");

  //Get calibration from another parameter file
  std::string vimheConfig = path + "/test/include_test/vimhe_config.xml";

  boost::property_tree::read_info(vimheConfig, propertyTree);

  solverParameters.twi_init_stddev = propertyTree.get<double>("vimhe.twi_init_stddev");
  solverParameters.qwi_init_stddev = propertyTree.get<double>("vimhe.qwi_init_stddev");
  solverParameters.vwi_init_stddev = propertyTree.get<double>("vimhe.vwi_init_stddev");
  solverParameters.ba_init_stddev = propertyTree.get<double>("vimhe.ba_init_stddev");
  solverParameters.bg_init_stddev = propertyTree.get<double>("vimhe.bg_init_stddev");
  aprilTagParameters.tagMeasCalibration_.ftr_init_stddev_ = propertyTree.get<double>("vimhe.ftr_init_stddev");
  aprilTagParameters.t_c_i_init_stddev = propertyTree.get<double>("vimhe.t_c_i_init_stddev");
  aprilTagParameters.q_c_i_init_stddev = propertyTree.get<double>("vimhe.q_c_i_init_stddev");
  aprilTagParameters.tagMeasCalibration_.tagSize_ = propertyTree.get<double>("vimhe.tagSize");
  solverParameters.imuCalibration_.gravityMagnitude_ = propertyTree.get<double>("vimhe.gravityMagnitude");

  solverParameters.wi_stddev_ = propertyTree.get<double>("vimhe.wi_stddev");
  solverParameters.ai_stddev_ = propertyTree.get<double>("vimhe.ai_stddev");
  solverParameters.bg_stddev_ = propertyTree.get<double>("vimhe.bg_stddev");
  solverParameters.ba_stddev_ = propertyTree.get<double>("vimhe.ba_stddev");
  aprilTagParameters.tag_corner_stddev_ = propertyTree.get<double>("vimhe.tag_corner_stddev");

  solverParameters.initialize();
  aprilTagParameters.initialize();

  T_c_i.trans(0) = propertyTree.get<double>("vimhe.T_c_i.px");
  T_c_i.trans(1) = propertyTree.get<double>("vimhe.T_c_i.py");
  T_c_i.trans(2) = propertyTree.get<double>("vimhe.T_c_i.pz");
  T_c_i.rot.w() = propertyTree.get<double>("vimhe.T_c_i.qw");
  T_c_i.rot.x() = propertyTree.get<double>("vimhe.T_c_i.qx");
  T_c_i.rot.y() = propertyTree.get<double>("vimhe.T_c_i.qy");
  T_c_i.rot.z() = propertyTree.get<double>("vimhe.T_c_i.qz");
  T_c_i.print("T_c_i in");

  //MHE solution
  const std::string bag_in = ros::package::getPath("confusion") + "/example/vi_test.bag";
//	std::deque<ImuMeas> imuMeasVec;
//	std::vector<TagMeas> tagMeasVec;
  std::deque<std::shared_ptr<ImuMeas>> imuMeasVec;
  std::vector<std::shared_ptr<TagMeas>> tagMeasVec;

//	initializeProjMat();
  importDataFromBag(bag_in,
                    imuMeasVec,
                    tagMeasVec,
                    solverParameters.imuCalibration_,
                    aprilTagParameters.tagMeasCalibration_,
                    gravity_rot,
                    aprilTagParameters.tagMeasCalibration_.projMat_,
                    T_c_i);

//std::cout << "projMat:\n" << solverParameters.tagMeasCalibration_.projMat_ << std::endl;

  runConFusor(imuMeasVec, tagMeasVec);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  srand(time(NULL));
  std::cout.precision(12);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

