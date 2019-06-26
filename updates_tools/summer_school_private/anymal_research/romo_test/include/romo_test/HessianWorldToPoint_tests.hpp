/*
 * HessianWorldToPoint_tests.cpp
 *
 *  Created on: Jun 25, 2017
 *      Author: Dario Bellicoso
 */

// romo
#include "romo/FiniteDifferences.hpp"

#ifndef ROMO_TEST_HES_ZERO_TOL
#define ROMO_TEST_HES_ZERO_TOL 1.0e-7
#endif

namespace romo_test {

template<typename RobotModelFixture_>
class HessianWorldToPointTests : public RobotModelFixture_ {
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

  void testCalcTranslationHessianWorldToBodyInWorldFrame() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the robot model;
    auto& robotModel = *this->getModelPtr();

    for (const auto& body : this->getModelPtr()->getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      for (unsigned int qIndex=0; qIndex<robotModel.getDofCount(); ++qIndex) {
        {
          // Compute the hessian through finite differences.
          Eigen::MatrixXd hessianTranslationFiniteDifferences = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());

          romo::finite_differences::estimateTranslationHessianWorldToBodyForState(
              robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD, qIndex, hessianTranslationFiniteDifferences);

          // Compute the hessian analytically. We need to set back the state that was used to compute the finite differences hessian.
          robotModel.setState(*this->getStatePtr(), true);
          Eigen::MatrixXd hessianTranslationAnalytic = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
          robotModel.getHessianTranslationWorldToBodyForState(
              hessianTranslationAnalytic, qIndex,
              branch, node, RobotModelType::CoordinateFrameEnum::WORLD);

          // Compare the two hessians
          std::string msg = "getTranslationalHessianWorldToBodyInWorldFrameForState for limb " + std::to_string(qIndex) + " and state " +  std::to_string(qIndex);
          KINDR_ASSERT_DOUBLE_MX_EQ_ZT(hessianTranslationFiniteDifferences, hessianTranslationAnalytic, 1.0, msg, ROMO_TEST_HES_ZERO_TOL);
        }

      }
    }
  }

  void testCalcRotationHessianWorldToBodyInWorldFrame() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the robot model;
    auto& robotModel = *this->getModelPtr();

    for (const auto& body : this->getModelPtr()->getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      for (unsigned int qIndex=0; qIndex<robotModel.getDofCount(); ++qIndex) {
        this->setRandomStateToModel();

        // Compute the hessian through finite differences.
        Eigen::MatrixXd hessianRotationFiniteDifferences = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
        romo::finite_differences::estimateRotationHessianWorldToBodyForState(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD, qIndex, hessianRotationFiniteDifferences);

        // Compute the hessian analytically. We need to set back the state that was used to compute the finite differences hessian.
        robotModel.setState(*this->getStatePtr(), true);
        Eigen::MatrixXd hessianRotationAnalytic = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
        robotModel.getHessianRotationWorldToBodyForState(
            hessianRotationAnalytic, qIndex,
            branch, node, RobotModelType::CoordinateFrameEnum::WORLD);

        // Compare the two hessians
        std::string msg = "getRotationHessianWorldToBodyInWorldFrameForState for limb " + std::to_string(qIndex) + " and state " +  std::to_string(qIndex);
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(hessianRotationFiniteDifferences, hessianRotationAnalytic, 1.0, msg, ROMO_TEST_HES_ZERO_TOL);
      }
    }
  }

  void testCalcSpatialHessianWorldToBodyInWorldFrame() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the robot model;
    auto& robotModel = *this->getModelPtr();

    for (const auto& body : this->getModelPtr()->getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      for (unsigned int qIndex=0; qIndex<robotModel.getDofCount(); ++qIndex) {
        this->setRandomStateToModel();

        // Compute the hessian through finite differences.
        Eigen::MatrixXd hessianTranslationFiniteDifferences = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
        romo::finite_differences::estimateTranslationHessianWorldToBodyForState(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD, qIndex, hessianTranslationFiniteDifferences);

        Eigen::MatrixXd hessianRotationFiniteDifferences = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
        romo::finite_differences::estimateRotationHessianWorldToBodyForState(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD, qIndex, hessianRotationFiniteDifferences);

        Eigen::MatrixXd hessianSpatialFiniteDifferences = Eigen::MatrixXd::Zero(6, robotModel.getDofCount());
        hessianSpatialFiniteDifferences.topRows(3) = hessianRotationFiniteDifferences;
        hessianSpatialFiniteDifferences.bottomRows(3) = hessianTranslationFiniteDifferences;

        // Compute the hessian analytically. We need to set back the state that was used to compute the finite differences hessian.
        robotModel.setState(*this->getStatePtr(), true);
        Eigen::MatrixXd hessianSpatialAnalytic = Eigen::MatrixXd::Zero(6, robotModel.getDofCount());
        robotModel.getHessianSpatialWorldToBodyForState(
            hessianSpatialAnalytic, qIndex,
            branch, node, RobotModelType::CoordinateFrameEnum::WORLD);

        // Compare the two hessians
        std::string msg = "getSpatialHessianWorldToBodyInWorldFrameForState for limb " + std::to_string(qIndex) + " and state " +  std::to_string(qIndex);
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(hessianSpatialFiniteDifferences, hessianSpatialAnalytic, 1.0, msg, ROMO_TEST_HES_ZERO_TOL);
      }
    }
  }

  void testCalcTranslationHessianWorldToComInWorldFrame() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the robot model;
    auto& robotModel = *this->getModelPtr();

    const int nU = robotModel.getState().getNumberOfGeneralizedVelocities();
    Eigen::MatrixXd hessianAnalytic;
    hessianAnalytic.resize(3, nU);

    Eigen::MatrixXd hessianFiniteDifferences;
    hessianFiniteDifferences.resize(3, nU);

    for (unsigned int q_j=0; q_j<nU; ++q_j) {
      std::string test = "test with q_j: " + std::to_string(q_j);

      hessianFiniteDifferences.setZero();
      hessianAnalytic.setZero();

      romo::finite_differences::estimateHessianWorldToComInWorldFrameForState(
          robotModel, q_j, hessianFiniteDifferences);
      robotModel.getHessianTranslationWorldToComForState(
          hessianAnalytic, q_j, RobotModelType::CoordinateFrameEnum::WORLD);

      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(hessianAnalytic, hessianFiniteDifferences, 1.0, test, ROMO_TEST_HES_ZERO_TOL);
    }
  }

};


TYPED_TEST_CASE(HessianWorldToPointTests, FIXTURE_TEST_TYPE);

TYPED_TEST(HessianWorldToPointTests, CalcTranslationHessianWorldToBodyInWorldFrame) {
  this->testCalcTranslationHessianWorldToBodyInWorldFrame();
}

TYPED_TEST(HessianWorldToPointTests, CalcRotationHessianWorldToBodyInWorldFrame) {
  this->testCalcRotationHessianWorldToBodyInWorldFrame();
}

TYPED_TEST(HessianWorldToPointTests, CalcSpatialHessianWorldToBodyInWorldFrame) {
  this->testCalcSpatialHessianWorldToBodyInWorldFrame();
}

TYPED_TEST(HessianWorldToPointTests, CalcSpatialHessianWorldToComInWorldFrame) {
  this->testCalcTranslationHessianWorldToComInWorldFrame();
}


}
