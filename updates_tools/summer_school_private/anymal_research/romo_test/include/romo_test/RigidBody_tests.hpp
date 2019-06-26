/*
 * RigidBody_tests.hpp
 *
 *  Created on: Jun 25, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

namespace romo_test {

template<typename RobotModelFixture_>
class RigidBodyTests : public RobotModelFixture_ {
 private:
  using Base = RobotModelFixture_;

 protected:
  using RobotModelType = typename Base::RobotModelType;
  using RobotStateType = typename RobotModelType::RobotState;

  using BodyEnum = typename RobotModelType::BodyEnum;
  using BodyNodeEnum = typename RobotModelType::BodyNodeEnum;
  using CoordinateFrameEnum = typename RobotModelType::CoordinateFrameEnum;
  using BranchEnum = typename RobotModelType::BranchEnum;
  using LimbEnum = typename RobotModelType::LimbEnum;
  using ContactEnum = typename RobotModelType::ContactEnum;
  using ContactState = typename RobotModelType::ContactState;

  void testRigidBodyPosition() {
    // Init the model and state.
    this->initWithRandomState();

    auto& robotModel = *this->getModelPtr();

    for (const auto& body : robotModel.getBodyContainer()) {
      Eigen::Vector3d positionWorldToNullPointOnBodyInWorldFrame =
          body->getPositionWorldToPointOnBody(
              Eigen::Vector3d::Zero(), RobotModelType::CoordinateFrameEnum::WORLD);

      Eigen::Vector3d positionWorldToBodyInWorldFrame = body->getPositionWorldToBody(RobotModelType::CoordinateFrameEnum::WORLD);

      std::string msg = "";
      KINDR_ASSERT_DOUBLE_MX_EQ(positionWorldToBodyInWorldFrame, positionWorldToNullPointOnBodyInWorldFrame, 1.0e-10, msg);
    }
  }
};


TYPED_TEST_CASE(RigidBodyTests, FIXTURE_TEST_TYPE);

TYPED_TEST(RigidBodyTests, TestPositions) {
  this->testRigidBodyPosition();
}

}
