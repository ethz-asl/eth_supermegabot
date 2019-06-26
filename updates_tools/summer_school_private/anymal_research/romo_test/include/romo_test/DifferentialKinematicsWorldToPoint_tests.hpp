/*!
* @file    DifferentialKinematicsWorldToPoint_tests.hpp
* @author  Dario Bellicoso
* @date    Jun, 2017
*/

#pragma once

// romo
#include "romo/FiniteDifferences.hpp"

#ifndef ROMO_TEST_DKIN_ZERO_TOL
#define ROMO_TEST_DKIN_ZERO_TOL 1.0e-7
#endif


namespace romo_test {

template<typename RobotModelFixture_>
class DifferentialKinematicsWorldToPointTests : public RobotModelFixture_ {
 private:
  using Base = RobotModelFixture_;

 protected:
  using RobotModelType = typename Base::RobotModelType;
  using RobotStateType = typename Base::RobotModelType::RobotState;

  using BodyEnum = typename Base::RobotModelType::BodyEnum;
  using BodyNodeEnum = typename Base::RobotModelType::BodyNodeEnum;
  using CoordinateFrameEnum = typename Base::RobotModelType::CoordinateFrameEnum;
  using BranchEnum = typename Base::RobotModelType::BranchEnum;
  using LimbEnum = typename Base::RobotModelType::LimbEnum;
  using ContactEnum = typename Base::RobotModelType::ContactEnum;
  using ContactState = typename Base::RobotModelType::ContactState;

  void testTorsoTwist() {
    this->initWithRandomState();
    auto& robotModel = *this->getModelPtr();
    auto& state = robotModel.getState();

    {
      const Eigen::Vector3d linearVelocityBaseInWorldFrameComputed =
          robotModel.getLinearVelocityWorldToBody(
              RobotModelType::BranchEnum::BASE, RobotModelType::BodyNodeEnum::BASE,
              RobotModelType::CoordinateFrameEnum::WORLD);
      const Eigen::Vector3d linearVelocityBaseInWorldFrameExpected =
          state.getLinearVelocityBaseInWorldFrame().toImplementation();

      const std::string msg = "testTorsoTwist: linear velocity in world frame";
      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(linearVelocityBaseInWorldFrameComputed, linearVelocityBaseInWorldFrameExpected, 1.0e-10, msg, ROMO_TEST_DKIN_ZERO_TOL);
    }

    {
      const Eigen::Vector3d linearVelocityBaseInBaseFrameComputed =
          robotModel.getLinearVelocityWorldToBody(
              RobotModelType::BranchEnum::BASE, RobotModelType::BodyNodeEnum::BASE,
              RobotModelType::CoordinateFrameEnum::BASE);
      const Eigen::Vector3d linearVelocityBaseInBaseFrameExpected =
          state.getOrientationBaseToWorld().inverseRotate(
              state.getLinearVelocityBaseInWorldFrame()).toImplementation();

      const std::string msg = "testTorsoTwist: linear velocity in base frame";
      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(linearVelocityBaseInBaseFrameComputed, linearVelocityBaseInBaseFrameExpected, 1.0e-10, msg, ROMO_TEST_DKIN_ZERO_TOL);
    }

    {
      const Eigen::Vector3d angularVelocityBaseInWorldFrameComputed =
          robotModel.getAngularVelocityWorldToBody(
              RobotModelType::BranchEnum::BASE, RobotModelType::BodyNodeEnum::BASE,
              RobotModelType::CoordinateFrameEnum::WORLD);
      const Eigen::Vector3d angularVelocityBaseInWorldFrameExpected =
          state.getOrientationBaseToWorld().rotate(
              state.getAngularVelocityBaseInBaseFrame()).toImplementation();

      const std::string msg = "testTorsoTwist: angular velocity in world frame";
      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(angularVelocityBaseInWorldFrameComputed, angularVelocityBaseInWorldFrameExpected, 1.0e-10, msg, ROMO_TEST_DKIN_ZERO_TOL);
    }

    {
      const Eigen::Vector3d angularVelocityBaseInBaseFrameComputed =
          robotModel.getAngularVelocityWorldToBody(
              RobotModelType::BranchEnum::BASE, RobotModelType::BodyNodeEnum::BASE,
              RobotModelType::CoordinateFrameEnum::BASE);
      const Eigen::Vector3d angularVelocityBaseInBaseFrameExpected =
          state.getAngularVelocityBaseInBaseFrame().toImplementation();

      const std::string msg = "testTorsoTwist: angular velocity in base frame";
      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(angularVelocityBaseInBaseFrameComputed, angularVelocityBaseInBaseFrameExpected, 1.0e-10, msg, ROMO_TEST_DKIN_ZERO_TOL);
    }
  }

  void testBodyLinearVelocity() {
    this->initWithRandomState();
    auto& robotModel = *this->getModelPtr();

    for (const auto& body : this->getModelPtr()->getBodyContainer()) {
      const auto branchEnum = body->getBranchEnum();
      const auto bodyNodeEnum = body->getBodyNodeEnum();

      {
        // Get the linear velocity from the implementation methods.
        Eigen::Vector3d linearVelocityComputed = robotModel.getLinearVelocityWorldToBody(
            branchEnum, bodyNodeEnum, RobotModelType::CoordinateFrameEnum::WORLD);

        // Get the linear velocity from J*dq.
        Eigen::MatrixXd jacobianTranslationWorldToBody = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
        robotModel.getJacobianTranslationWorldToBody(
            jacobianTranslationWorldToBody, branchEnum, bodyNodeEnum,
            RobotModelType::CoordinateFrameEnum::WORLD);
        Eigen::Vector3d linearVelocityExpected = jacobianTranslationWorldToBody * robotModel.getState().getGeneralizedVelocities();

        Eigen::Vector3d linearVelocityFiniteDifferences;
        romo::finite_differences::estimateLinearVelocityBody(
            robotModel, branchEnum, bodyNodeEnum,
            RobotModelType::CoordinateFrameEnum::WORLD,
            linearVelocityFiniteDifferences);

        // Compare.
        std::string msg = "testBodyVelocity in world frame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(linearVelocityComputed, linearVelocityExpected, 1.0, msg, ROMO_TEST_DKIN_ZERO_TOL);
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(linearVelocityComputed, linearVelocityFiniteDifferences, 1.0, msg, ROMO_TEST_DKIN_ZERO_TOL);
      }

      {
        // Get the linear velocity from the implementation methods.
        Eigen::Vector3d linearVelocityComputed = this->getModelPtr()->getLinearVelocityWorldToBody(
            branchEnum, bodyNodeEnum, RobotModelType::CoordinateFrameEnum::BASE);

        // Get the linear velocity from J*dq.
        Eigen::MatrixXd jacobianTranslationWorldToBody = Eigen::MatrixXd::Zero(3, this->getModelPtr()->getDofCount());
        this->getModelPtr()->getJacobianTranslationWorldToBody(
            jacobianTranslationWorldToBody, branchEnum, bodyNodeEnum, RobotModelType::CoordinateFrameEnum::BASE);
        Eigen::Vector3d linearVelocityExpected = jacobianTranslationWorldToBody * this->getModelPtr()->getState().getGeneralizedVelocities();

        // Compare.
        std::string msg = "testBodyVelocity in base frame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(linearVelocityComputed, linearVelocityExpected, 1.0, msg, ROMO_TEST_DKIN_ZERO_TOL);
      }
    }
  }

  void testBodyAngularVelocity() {
    this->initWithRandomState();

    for (const auto& body : this->getModelPtr()->getBodyContainer()) {
      const auto branchEnum = body->getBranchEnum();
      const auto bodyNodeEnum = body->getBodyNodeEnum();

      {
        // Get the angular velocity from the implementation methods.
        Eigen::Vector3d angularVelocityComputed = this->getModelPtr()->getAngularVelocityWorldToBody(
            branchEnum, bodyNodeEnum, RobotModelType::CoordinateFrameEnum::WORLD);

        // Get the angular velocity from J*dq.
        Eigen::MatrixXd jacobianRotationalWorldToBody = Eigen::MatrixXd::Zero(3, this->getModelPtr()->getDofCount());
        this->getModelPtr()->getJacobianRotationWorldToBody(
            jacobianRotationalWorldToBody, branchEnum, bodyNodeEnum, RobotModelType::CoordinateFrameEnum::WORLD);
        Eigen::Vector3d angularVelocityExpected = jacobianRotationalWorldToBody * this->getModelPtr()->getState().getGeneralizedVelocities();

        // Compare.
        std::string msg = "testAngularVelocityBody in world frame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(angularVelocityComputed, angularVelocityExpected, 1.0, msg, ROMO_TEST_DKIN_ZERO_TOL);
      }

      {
        // Get the angular velocity from the implementation methods.
        Eigen::Vector3d angularVelocityComputed = this->getModelPtr()->getAngularVelocityWorldToBody(
            branchEnum, bodyNodeEnum, RobotModelType::CoordinateFrameEnum::BASE);

        // Get the angular velocity from J*dq.
        Eigen::MatrixXd jacobianRotationalWorldToBody = Eigen::MatrixXd::Zero(3, this->getModelPtr()->getDofCount());
        this->getModelPtr()->getJacobianRotationWorldToBody(
            jacobianRotationalWorldToBody, branchEnum, bodyNodeEnum, RobotModelType::CoordinateFrameEnum::BASE);
        Eigen::Vector3d angularVelocityExpected = jacobianRotationalWorldToBody * this->getModelPtr()->getState().getGeneralizedVelocities();

        // Compare.
        std::string msg = "testAngularVelocityBody in base frame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(angularVelocityComputed, angularVelocityExpected, 1.0, msg, ROMO_TEST_DKIN_ZERO_TOL);
      }
    }
  }
};


TYPED_TEST_CASE(DifferentialKinematicsWorldToPointTests, FIXTURE_TEST_TYPE);

TYPED_TEST(DifferentialKinematicsWorldToPointTests, TorsoTwist) {
  this->testTorsoTwist();
}

TYPED_TEST(DifferentialKinematicsWorldToPointTests, BodyLinearVelocity) {
  this->testBodyLinearVelocity();
}

TYPED_TEST(DifferentialKinematicsWorldToPointTests, BodyAngularVelocity) {
  this->testBodyAngularVelocity();
}

}
