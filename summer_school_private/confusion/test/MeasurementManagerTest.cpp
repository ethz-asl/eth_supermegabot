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

#include <iostream>

#include <gtest/gtest.h>

#include "confusion/ConFusor.h"
#include "confusion/ProcessChain.h"
#include "confusion/UpdateMeasurement.h"
#include "confusion/MeasurementManager.h"
#include "confusion/State.h"
#include "confusion/StateVector.h"
#include "confusion/StaticParameterVector.h"

using namespace confusion;

class TestUpdateMeas : public UpdateMeasurement {
 public:
  TestUpdateMeas(int measType, double t) : UpdateMeasurement(measType, t, "test", false) {}

  ~TestUpdateMeas() {}

  bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                          std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                          std::vector<size_t> &stateParameterIndexVector,
                          std::vector<double *> &staticParameterDataVector) {
    return true;
  }

  int residualDimension() { return 1; }
};

class TestProcessChain : public ProcessChain {
 public:
  TestProcessChain() : ProcessChain("test", false) {}

  bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                          std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                          std::vector<size_t> &stateParameterIndexVector,
                          std::vector<double *> &staticParameterDataVector) {
    return true;
  }

  int residualDimension() { return 1; }
};

class TestState : public State {
 public:
  TestState(double t, int numProcessSensors, int numUpdateSensors) : State(t, numProcessSensors, numUpdateSensors) {

    for (int i = 0; i < numProcessSensors; ++i) {
      processChains_[i] = std::make_shared<TestProcessChain>();
    }
    parameters_.push_back(confusion::Parameter(param.data(), 3, "test"));
  }

  TestState(const TestState *in) : State(in->t_, in->numProcessSensors_, in->numUpdateSensors_) {
    for (int i = 0; i < in->numProcessSensors_; ++i) {
      processChains_[i] = std::make_shared<TestProcessChain>();
    }
    parameters_.push_back(in->parameters_[0].clone(param.data()));
  }

  bool initFirstState(const std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>> &processMeasBuffer,
                      const std::vector<std::deque<std::shared_ptr<UpdateMeasurement>>> &updateMeasBuffer,
                      StaticParameterVector &staticParameters) {
    bool res = false;
    for (int i = 0; i < numUpdateSensors_; ++i) {
      if (updateMeasBuffer[i].empty())
        continue;

      t_ = updateMeasBuffer[i].front()->t();
      res = true;
//      std::cout << "Created first state at t=" << t_ << std::endl;
      break;
    }

    return res;
  }

  std::shared_ptr<State> createNextState(
      const std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>> &processMeasBuffer,
      const std::vector<std::deque<std::shared_ptr<UpdateMeasurement>>> &updateMeasBuffer,
      StaticParameterVector &staticParameters) {
    double t_out;
    bool res = false;
    for (int i = 0; i < numUpdateSensors_; ++i) {
      if (updateMeasBuffer[i].empty())
        continue;

      t_out = updateMeasBuffer[i].front()->t();
      res = true;
      break;
    }

    if (!res)
      return nullptr;

    auto stateOut = std::make_shared<TestState>(this);
    stateOut->t_ = t_out;
//    std::cout << "Created next state at t=" << stateOut->t_ << std::endl;

    return stateOut;
  }

 private:
  Eigen::Vector3d param;
};

// Creates clean setup of Confusor, with one update measurements at t=0,1,2, and process measurements from 0:0.25:2.75
void setupConfusor(ConFusor &confusor, const int numProcessSensors, const int numUpdateSensors) {
  int freq = 4;
  for (int i = 0; i <= 2 * freq; i += 1) {
    double t = i / static_cast<double>(freq);
    for (int sensor = 0; sensor < numProcessSensors; ++sensor) {
      confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(sensor, t));
    }
    if (i % freq == 0) {
      for (int sensor = 0; sensor < numUpdateSensors; ++sensor) {
        confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(sensor, t));
      }
      confusor.assignMeasurements();
    }
  }
}

TEST(SetupConfusorTest, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  double tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  EXPECT_EQ(confusor.stateVector_.size(), 3);
  EXPECT_FALSE(confusor.stateVector_[0]->processChains_[0]->measurements_.empty());
  EXPECT_FALSE(confusor.stateVector_[0]->updateMeasurements_[0].empty());
  EXPECT_FALSE(confusor.stateVector_[1]->processChains_[0]->measurements_.empty());
  EXPECT_FALSE(confusor.stateVector_[1]->updateMeasurements_[0].empty());
  EXPECT_TRUE(confusor.stateVector_[2]->processChains_[0]->measurements_.empty());
  EXPECT_FALSE(confusor.stateVector_[2]->updateMeasurements_[0].empty());
}

TEST(SetupConfusorTestTwoUpdates, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 2;
  double tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  EXPECT_EQ(confusor.stateVector_.size(), 3);
  EXPECT_FALSE(confusor.stateVector_[0]->processChains_[0]->measurements_.empty());
  EXPECT_FALSE(confusor.stateVector_[0]->updateMeasurements_[0].empty());
  EXPECT_FALSE(confusor.stateVector_[0]->updateMeasurements_[1].empty());
  EXPECT_FALSE(confusor.stateVector_[1]->processChains_[0]->measurements_.empty());
  EXPECT_FALSE(confusor.stateVector_[1]->updateMeasurements_[0].empty());
  EXPECT_FALSE(confusor.stateVector_[1]->updateMeasurements_[1].empty());
  EXPECT_TRUE(confusor.stateVector_[2]->processChains_[0]->measurements_.empty());
  EXPECT_FALSE(confusor.stateVector_[2]->updateMeasurements_[0].empty());
  EXPECT_FALSE(confusor.stateVector_[2]->updateMeasurements_[1].empty());
}


// Last process measurement -> don't add
TEST(LastProcessMeasurement, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  double tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  double tProcess = 2.5;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess));
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess + 0.25));
  confusor.assignMeasurements();

  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 3));
  confusor.assignMeasurements();

  // Expect that the last process measurement at 2.75 was not added
  EXPECT_TRUE(confusor.stateVector_[3]->processChains_[0]->measurements_.empty());
//  EXPECT_DOUBLE_EQ(confusor.stateVector_[2]->processChains_[0]->measurements_.back()->t(), tProcess);
}

// Two update measurements with one call to assignMeasurement()
TEST(TwoUpdateMeasurementsWithOneAssign, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 3));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));
  confusor.assignMeasurements();

  ASSERT_EQ(confusor.stateVector_.size(),5);
  EXPECT_FALSE(confusor.stateVector_[3]->updateMeasurements_[0].empty());
  EXPECT_FALSE(confusor.stateVector_[4]->updateMeasurements_[0].empty());
}

// Adding a second update measurement to an older state -> add to old state
TEST(SecondUpdateMeasurement, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 3));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 5));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 6));

  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));

  confusor.assignMeasurements();

  EXPECT_EQ(confusor.stateVector_[4]->updateMeasurements_[0].size(), 2);
  EXPECT_EQ(confusor.stateVector_[5]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[6]->updateMeasurements_[0].size(), 1);
}

// Adding an old process measurement -> add to old state(s)
TEST(OldProcessMeasurement, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  for (double tUpdate = 3; tUpdate <= 6; ++tUpdate) {
    confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, tUpdate));
  }

  // Add old process measurement, check that this gets added to state 2
  double tProcess = 3.5;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess));
  confusor.assignMeasurements();

  // Add additional process measurement
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, 5));
  confusor.assignMeasurements();

  EXPECT_DOUBLE_EQ(confusor.stateVector_[3]->processChains_[0]->measurements_.back()->t(), tProcess);
}


// Two update measurements, with one delayed
TEST(OneDelayedUpdateMeasurement, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 2;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 3));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));
  confusor.assignMeasurements();

  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(1, 3));
  confusor.assignMeasurements();

  EXPECT_FALSE(confusor.stateVector_[3]->updateMeasurements_[0].empty());
  EXPECT_FALSE(confusor.stateVector_[3]->updateMeasurements_[1].empty());
  EXPECT_FALSE(confusor.stateVector_[4]->updateMeasurements_[0].empty());
  EXPECT_TRUE (confusor.stateVector_[4]->updateMeasurements_[1].empty());
}


// No process measurements between two states -> add process measurement to both states after it
TEST(NoProcessMeasurementBetweenStates, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  double tProcess = 2.5;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 3));
  confusor.assignMeasurements();

  // No process measurements between t = 3 and 4
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));
  confusor.assignMeasurements();

  double tProcessLast = 4.5;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcessLast));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 5));
  confusor.assignMeasurements();

  // Add additional process measurement
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, 5.5));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 6));
  confusor.assignMeasurements();

  // Expect that process measurement at t = 2.5 is added to states 2, 3 and 4, but not to state 5
  EXPECT_DOUBLE_EQ(confusor.stateVector_[2]->processChains_[0]->measurements_.back()->t(), tProcess);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[3]->processChains_[0]->measurements_.front()->t(), tProcess);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[3]->processChains_[0]->measurements_.back()->t(), tProcess);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[4]->processChains_[0]->measurements_.front()->t(), tProcess);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[4]->processChains_[0]->measurements_.back()->t(), tProcessLast);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[5]->processChains_[0]->measurements_.front()->t(), tProcessLast);
}


// Process meas aligned with state -> add only to newer state
TEST(ProcessMeasAligned, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  tState = 3;
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, tState));

  double tProcess = tState;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess));
  confusor.assignMeasurements();

  // Add additional process measurement, to circumvent error in LastProcessMeasurement
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, 3.5));
  confusor.assignMeasurements();

  // Check that process measurement was only assigned to state 3, not to state 2
  EXPECT_GT(std::abs(confusor.stateVector_[2]->processChains_[0]->measurements_.back()->t() - tProcess), 0.1);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[3]->processChains_[0]->measurements_.front()->t(), tProcess);
}


// Process meas not aligned with state -> add last measurement both to previous and next state
TEST(ProcessMeasNotAligned, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  tState = 3;
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, tState));

  double tProcess = tState - 0.1;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess));
  confusor.assignMeasurements();

  // Add additional process measurement, to circumvent error in LastProcessMeasurement
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, 3.5));
  confusor.assignMeasurements();

  // Check that process measurement was assigned to both state 2 and 3
  EXPECT_DOUBLE_EQ(confusor.stateVector_[2]->processChains_[0]->measurements_.back()->t(), tProcess);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[3]->processChains_[0]->measurements_.front()->t(), tProcess);
}


// Update measurement does not arrive in sequence -> add the measurements correctly
TEST(UpdateOutOfSequence, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  for (tState = 3; tState <= 4; ++tState) {
    confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, tState));
  }

  // Add update measurements out of sequence
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));
  confusor.assignMeasurements();
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 3));
  confusor.assignMeasurements();
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, -1));
  confusor.assignMeasurements();

  EXPECT_EQ(confusor.stateVector_[0]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[1]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[2]->updateMeasurements_[0].size(), 1);
}

// Two update measurements arriving in alternating sequence -> add the measurements correctly
TEST(TwoUpdateSensors, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 2;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, 1);

  // Add update measurements of second type
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(1, 3));
  confusor.assignMeasurements();
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));
  confusor.assignMeasurements();
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(1, 5));
  confusor.assignMeasurements();

  ASSERT_EQ(confusor.stateVector_.size(), 6);
  EXPECT_EQ(confusor.stateVector_[4]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[4]->updateMeasurements_[1].size(), 0);
  EXPECT_EQ(confusor.stateVector_[5]->updateMeasurements_[0].size(), 0);
  EXPECT_EQ(confusor.stateVector_[5]->updateMeasurements_[1].size(), 1);
}

// Process measurements arrive in sequence, but one sensor delayed -> accept
TEST(OneOfTwoProcessSensorsDelayed, ShouldPass) {
  const int numProcessSensors = 2;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  tState = 3;
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, tState));

  double tProcess = 2.5;
  int sensor = 0;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(sensor, tProcess));
  confusor.assignMeasurements();

  tState = 4;
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, tState));

  tProcess = 3.5;
  sensor = 0;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(sensor, tProcess));
  confusor.assignMeasurements();

  // Add delayed second sensor
  tProcess = 2.3;
  sensor = 1;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(sensor, tProcess));
  confusor.assignMeasurements();

  tProcess = 2.6;
  sensor = 1;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(sensor, tProcess));
  confusor.assignMeasurements();

  // Add additional process measurements, to circumvent error in LastProcessMeasurement
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 5));
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, 4.5));
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(1, 4.5));
  confusor.assignMeasurements();

  // Check that signals of second sensor were added to state 2
  EXPECT_DOUBLE_EQ(confusor.stateVector_[2]->processChains_[1]->measurements_[1]->t(), 2.3);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[2]->processChains_[1]->measurements_.back()->t(), 2.6);
  // Check that no signals of second sensor were added to state 3 (except the one immediately preceding it)
  EXPECT_DOUBLE_EQ(confusor.stateVector_[3]->processChains_[1]->measurements_.front()->t(), 2.6);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[3]->processChains_[1]->measurements_.back()->t(), 2.6);
}


// Process measurement (per sensor) does not arrive in sequence -> throw error
TEST(ProcessSensorOutOfSequence, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 3));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 4));

  double tProcess = 3.0;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess));
  confusor.assignMeasurements();

  tProcess = 3.6;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess));
  confusor.assignMeasurements();

  tProcess = 3.3;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess));
  confusor.assignMeasurements();

  // Check that process measurement at 3.3 was not added
  EXPECT_DOUBLE_EQ(confusor.stateVector_[3]->processChains_[0]->measurements_.front()->t(), 3);
  EXPECT_DOUBLE_EQ(confusor.stateVector_[3]->processChains_[0]->measurements_.back()->t(), 3.6);
  EXPECT_EQ(confusor.stateVector_[3]->processChains_[0]->measurements_.size(), 2);
}


// Process measurement in sequence is older than oldest state -> throw out (only way to trigger throw out)
TEST(TooOldProcessMeasurement, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 1; //! Start at 1 to fail on early process measurement later
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));

  // Set up confusor
  for (tState = 2; tState <= 3; tState++) {
    confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, tState));
    confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tState - 1));
    confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tState - 0.5));
    confusor.assignMeasurements();
  }

  testing::internal::CaptureStdout();

  // Add process measurement out of sequence
  double tProcess = 0.5;
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, tProcess));
  confusor.assignMeasurements();

  std::string output = testing::internal::GetCapturedStdout();
//    std::cout << "Captured stdout:\n" << output;

  // Expect error message
  EXPECT_FALSE(output.empty());
}


// TODO: Currently goes into infinite recursion because MeasurementManager.h l. 286
//      if (stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() < updateMeasBuffer_[i].front()->t())
// will always be true for i = 0 in this test
// createNextState() returns nullptr -> still add all states
TEST(createNextStateReturnsNullPtr, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 2;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, numUpdateSensors);

  // Add only update measurement for sensor 1 and trigger nullptr
  double tUpdate = 3;
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(1, tUpdate));
  confusor.assignMeasurements();

  tUpdate = 4;
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, tUpdate));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(1, tUpdate));
  confusor.assignMeasurements();

  EXPECT_TRUE(confusor.stateVector_[3]->updateMeasurements_[0].empty());
  EXPECT_FALSE(confusor.stateVector_[3]->updateMeasurements_[1].empty());
  EXPECT_FALSE(confusor.stateVector_[4]->updateMeasurements_[0].empty());
  EXPECT_FALSE(confusor.stateVector_[4]->updateMeasurements_[1].empty());
}

// Old asynchronous update measurement of another type arrives behind the last state -> create a new state and assign the measurement
TEST(CreateIntermediateState, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 2;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, 1);

  //Add one more process meas to set the process meas to ready
  confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(0, 2.1));
  confusor.assignMeasurements();
  ASSERT_TRUE(confusor.stateVector_[1]->processChains_[0]->ready());

  // Add update measurement of second type behind the last state
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(1, 1.5));
  confusor.assignMeasurements();

  EXPECT_EQ(confusor.stateVector_.size(), 4);

  //Check update measurements
  EXPECT_EQ(confusor.stateVector_[0]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[1]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[2]->updateMeasurements_[0].size(), 0);
  EXPECT_EQ(confusor.stateVector_[2]->updateMeasurements_[1].size(), 1);
  EXPECT_EQ(confusor.stateVector_[3]->updateMeasurements_[0].size(), 1);

  //Check process chains
  EXPECT_LE(confusor.stateVector_[1]->processChains_[0]->measurements_.back()->t(), confusor.stateVector_[2]->t());
  EXPECT_LE(confusor.stateVector_[2]->processChains_[0]->measurements_.front()->t(), confusor.stateVector_[2]->t());
  EXPECT_LE(confusor.stateVector_[2]->processChains_[0]->measurements_.back()->t(), confusor.stateVector_[3]->t());
  EXPECT_TRUE(confusor.stateVector_[1]->processChains_[0]->ready());
  EXPECT_TRUE(confusor.stateVector_[2]->processChains_[0]->ready());
  EXPECT_EQ(confusor.stateVector_[1]->processChains_[0]->tStart(), confusor.stateVector_[1]->t());
  EXPECT_EQ(confusor.stateVector_[1]->processChains_[0]->tEnd(), confusor.stateVector_[2]->t());
  EXPECT_EQ(confusor.stateVector_[2]->processChains_[0]->tStart(), confusor.stateVector_[2]->t());
  EXPECT_EQ(confusor.stateVector_[2]->processChains_[0]->tEnd(), confusor.stateVector_[3]->t());
}

TEST(CreateIntermediateStateNotReady, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 2;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, 1);

  ASSERT_FALSE(confusor.stateVector_[2]->processChains_[0]->ready());

  // Add update measurement of second type behind the last state
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(1, 1.5));
  confusor.assignMeasurements();

  EXPECT_EQ(confusor.stateVector_.size(), 4);

  //Check update measurements
  EXPECT_EQ(confusor.stateVector_[0]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[1]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[2]->updateMeasurements_[0].size(), 0);
  EXPECT_EQ(confusor.stateVector_[2]->updateMeasurements_[1].size(), 1);
  EXPECT_EQ(confusor.stateVector_[3]->updateMeasurements_[0].size(), 1);

  //Check process chains
  EXPECT_LE(confusor.stateVector_[1]->processChains_[0]->measurements_.back()->t(), confusor.stateVector_[2]->t());
  EXPECT_LE(confusor.stateVector_[2]->processChains_[0]->measurements_.front()->t(), confusor.stateVector_[2]->t());
  EXPECT_LE(confusor.stateVector_[2]->processChains_[0]->measurements_.back()->t(), confusor.stateVector_[3]->t());
  EXPECT_TRUE(confusor.stateVector_[1]->processChains_[0]->ready());
  EXPECT_FALSE(confusor.stateVector_[2]->processChains_[0]->ready());
  EXPECT_EQ(confusor.stateVector_[1]->processChains_[0]->tStart(), confusor.stateVector_[1]->t());
  EXPECT_EQ(confusor.stateVector_[1]->processChains_[0]->tEnd(), confusor.stateVector_[2]->t());
}

TEST(CreateIntermediateStateAssignNoProcessMeas, ShouldPass) {
  const int numProcessSensors = 1;
  const int numUpdateSensors = 1;
  int tState = 0;
  ConFusor confusor(std::make_shared<TestState>(tState, numProcessSensors, numUpdateSensors));
  setupConfusor(confusor, numProcessSensors, 1);

  // Add update measurement of second type behind the last state
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 3));
  confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(0, 2.5));
  confusor.assignMeasurements();

  EXPECT_EQ(confusor.stateVector_.size(), 5);

  //Check update measurements
  EXPECT_EQ(confusor.stateVector_[0]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[1]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[2]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[3]->updateMeasurements_[0].size(), 1);
  EXPECT_EQ(confusor.stateVector_[4]->updateMeasurements_[0].size(), 1);

  //Check process chains
  EXPECT_TRUE(confusor.stateVector_[2]->processChains_[0]->measurements_.empty());
  EXPECT_TRUE(confusor.stateVector_[3]->processChains_[0]->measurements_.empty());
  EXPECT_TRUE(confusor.stateVector_[4]->processChains_[0]->measurements_.empty());
  EXPECT_FALSE(confusor.stateVector_[2]->processChains_[0]->ready());
  EXPECT_FALSE(confusor.stateVector_[3]->processChains_[0]->ready());
  EXPECT_FALSE(confusor.stateVector_[4]->processChains_[0]->ready());
}

int main(int argc, char **argv) {
  srand(time(NULL));
  std::cout.precision(12);

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

