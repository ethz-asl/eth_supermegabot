/*
 * KinematicsWorldToBody_tests.hpp
 *
 *  Created on: Jun 26, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

namespace romo_test {

template<typename RobotFixtureType_>
class KinematicsWorldToBodyTests : public RobotFixtureType_ {
 private:
  using Base = RobotFixtureType_;

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

  //! Rotation matrices obtained from the model should be orthonormal.
  void testRotationMatrixOrthonormality() {
    // Initialize the model and set a random state.
    this->initWithRandomState();

    // Test the rotation matrices obtained for all bodies in the body container.
    for (const auto& body : this->getModelPtr()->getBodyContainer()) {
      const Eigen::Matrix3d& rotationMatrix = body->getOrientationWorldToBody();

      // The determinant should be 1.0.
      const double determinant = rotationMatrix.determinant();
      EXPECT_NEAR(1.0, determinant, 1e-10);

      // The rotation matrix should be orthogonal.
      Eigen::MatrixXd identity = rotationMatrix.transpose()*rotationMatrix;
      const std::string msg = "testRotationMatrixOrthonormality for body named " + body->getName();
      KINDR_ASSERT_DOUBLE_MX_EQ(identity, Eigen::Matrix3d::Identity(), 1.0, msg);
    }
  }

  //! The base orientation obtained from the model and from the state should be the same.
  void testBaseOrientationStateAndModel() {
    // Initialize the model and set a random state.
    this->initWithRandomState();

    // Compare orientations obtained from the state and from the model.
    romo::RotationMatrix orientationWorldToBaseFromModelKindr = romo::RotationMatrix(this->getModelPtr()->getOrientationWorldToBody(BodyEnum::BASE));
    romo::RotationMatrix orientationWorldToBaseFromStateKindr = romo::RotationMatrix(this->getModelPtr()->getState().getOrientationBaseToWorld().inverted());

    ASSERT_TRUE(this->getModelPtr()->getState().getOrientationBaseToWorld().inverted().isNear(orientationWorldToBaseFromModelKindr, 1.0e-3)) << "quaternion check";
    EXPECT_NEAR(0.0, orientationWorldToBaseFromModelKindr.getDisparityAngle(orientationWorldToBaseFromStateKindr), 1.0e-3);
  }

  //! Rotating a position vector by the state orientation and the model should yield the same result.
  void testRotatePosition() {
    // Initialize the model and set a random state.
    this->initWithRandomState();

    const Eigen::Vector3d positionA(-1.2, 3.4, 0.9);

    const auto orientationA = this->getStatePtr()->getOrientationBaseToWorld().inverted();
    const auto orientationAB = romo::RotationQuaternion(romo::EulerAnglesZyx(0.5, -0.8, 1.4));
    const auto orientationB = orientationAB*orientationA;

    this->getStatePtr()->setOrientationBaseToWorld(orientationB.inverted());
    this->getModelPtr()->setState(*this->getStatePtr(), true);
    Eigen::Vector3d modelPositionB = this->getModelPtr()->getOrientationWorldToBody(BodyEnum::BASE)*positionA;
    Eigen::Vector3d statePositionB = orientationB.rotate(positionA);

    EXPECT_NEAR(statePositionB.x(), modelPositionB.x(), 1.0e-3);
    EXPECT_NEAR(statePositionB.y(), modelPositionB.y(), 1.0e-3);
    EXPECT_NEAR(statePositionB.z(), modelPositionB.z(), 1.0e-3);
  }

};


TYPED_TEST_CASE(KinematicsWorldToBodyTests, FIXTURE_TEST_TYPE);

TYPED_TEST(KinematicsWorldToBodyTests, TestRotationOrthonormality) {
  this->testRotationMatrixOrthonormality();
}

TYPED_TEST(KinematicsWorldToBodyTests, TestBaseOrientationStateAndModel) {
  this->testBaseOrientationStateAndModel();
}

#ifndef ROMO_TEST_FIXED_BASE
TYPED_TEST(KinematicsWorldToBodyTests, TestRotatePosition) {
  this->testRotatePosition();
}
#endif

}
