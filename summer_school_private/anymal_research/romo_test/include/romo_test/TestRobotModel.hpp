/*!
* @file    TestRobotModel.hpp
* @author  Dario Bellicoso
* @date    Jun, 2017
*/

#pragma once

// gtest
#include <gtest/gtest.h>

// stl
#include <boost/filesystem.hpp>

// eigen
#include <Eigen/Core>

// robot model
#include "romo/RobotModel.hpp"

namespace romo_test {

template<typename RobotModel_>
class TestRobotModel : public ::testing::Test {
 protected:
  using RobotModelType = RobotModel_;
  using RobotStateType = typename RobotModelType::RobotState;
  using RobotModelTypePtr = std::shared_ptr<RobotModelType>;
  using RobotStateTypePtr = std::shared_ptr<RobotStateType>;

protected:
  using BodyEnum = typename RobotModelType::BodyEnum;
  using BodyNodeEnum = typename RobotModelType::BodyNodeEnum;
  using CoordinateFrameEnum = typename RobotModelType::CoordinateFrameEnum;
  using BranchEnum = typename RobotModelType::BranchEnum;
  using LimbEnum = typename RobotModelType::LimbEnum;
  using ContactEnum = typename RobotModelType::ContactEnum;
  using ContactState = typename RobotModelType::ContactState;

 public:
  TestRobotModel() :
    model_(),
    state_()
  { }

  virtual ~TestRobotModel() { }

  virtual void init() = 0;
  virtual void getRandomGeneralizedPositionsRbdl(Eigen::VectorXd& q) = 0;

  virtual void initWithRandomState() {
    // Init the model and state.
    this->init();
    this->setRandomStateToModel();
  }

  virtual void setRandomStateToModel() {
    // Set the state to a random one.
    this->setStateToRandom();

    // Update the model with the new state.
    this->getModelPtr()->setState(*this->getStatePtr(), true, true);
  }

  virtual void setStateToRandom() {
    // Set the state to a random one.
    this->getStatePtr()->setRandom();
    this->getStatePtr()->getJointPositions().toImplementation().setRandom();
    this->getStatePtr()->getJointVelocities().toImplementation().setRandom();
  }

  virtual void setRbdlQFromState(Eigen::VectorXd& rbdlQ, const RobotStateType& state) = 0;
  virtual void setStateFromRbdlQ(RobotStateType& state, const Eigen::VectorXd& rbdlQ) = 0;

  virtual RobotModelTypePtr getModelPtr() {
    return model_;
  }

  virtual RobotStateTypePtr getStatePtr() {
    return state_;
  }

 protected:
  RobotModelTypePtr model_;
  RobotStateTypePtr state_;
};

}
