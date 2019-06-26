/*
 * RomoTestFixtureExample.hpp
 *
 *  Created on: Jun 26, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// gtest
#include <gtest/gtest.h>

// kindr
#include "kindr/common/gtest_eigen.hpp"

// test robot model
#include "romo_test/TestRobotModel.hpp"

// romo rbdl
#include "romo/common/phys_typedefs.hpp"

/*
 * An example fixture class. Implement your test methods here and call them in your implemented model unit tests.
 */


namespace romo_test {

template<typename RobotModel_>
class RomoTestFixtureExample : public ::testing::Test, virtual public TestRobotModel<RobotModel_> {
 protected:
  using RobotModelType = RobotModel_;
  using RobotStateType = typename RobotModelType::RobotState;

  using BodyEnum = typename RobotModelType::BodyEnum;
  using BodyNodeEnum = typename RobotModelType::BodyNodeEnum;
  using CoordinateFrameEnum = typename RobotModelType::CoordinateFrameEnum;
  using BranchEnum = typename RobotModelType::BranchEnum;
  using LimbEnum = typename RobotModelType::LimbEnum;
  using ContactEnum = typename RobotModelType::ContactEnum;
  using ContactState = typename RobotModelType::ContactState;

  //! An example test method that can be called in an instantiation of this class.
  void testMe() {
    // Init the model and state.
    this->init();

    // Set the state to a random one.
    this->getStatePtr()->setRandom();

    // Update the model with the new state.
    this->getModelPtr()->setState(*this->getStatePtr(), true, true);

    /*
     * This set of instructions could have been replaced by calling:
     *  this->initWithRandomState();
     */


    // test something...


  }

};

}

