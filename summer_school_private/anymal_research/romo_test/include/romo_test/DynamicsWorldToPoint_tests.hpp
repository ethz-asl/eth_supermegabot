/*!
* @file    DynamicsWorldToPoint_tests.cpp
* @author  Dario Bellicoso
* @date    Sep, 2015
*/

#ifndef ROMO_TEST_DYN_ZERO_TOL
  #define ROMO_TEST_DYN_ZERO_TOL 1.0e-8
#endif

// romo
#include "romo/FiniteDifferences.hpp"

namespace romo_test {

template<typename RobotModelFixture_>
class DynamicsWorldToPointTests : virtual public RobotModelFixture_ {
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

  void testMassMatrix() {
    this->initWithRandomState();
    auto& robotModel = *this->getModelPtr();

    Eigen::MatrixXd massInertiaMatrixFiniteDifferences =
        Eigen::MatrixXd::Zero(robotModel.getDofCount(), robotModel.getDofCount());
    romo::finite_differences::estimateMassMatrix(robotModel, massInertiaMatrixFiniteDifferences);
    Eigen::MatrixXd massInertiaMatrixAnalytical =
        Eigen::MatrixXd::Zero(robotModel.getDofCount(), robotModel.getDofCount());
    robotModel.getMassInertiaMatrix(massInertiaMatrixAnalytical);

    std::string msg = "";
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(massInertiaMatrixFiniteDifferences, massInertiaMatrixAnalytical, 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);
  }

  void testGravityTerms() {
    this->initWithRandomState();
    auto& robotModel = *this->getModelPtr();

    Eigen::VectorXd gravityTermsFiniteDifferences = Eigen::VectorXd::Zero(robotModel.getDofCount());
    romo::finite_differences::estimateGravityTerms(robotModel, gravityTermsFiniteDifferences);

    Eigen::VectorXd gravityTermsAnalytical = Eigen::VectorXd::Zero(robotModel.getDofCount());
    robotModel.getGravityTerms(gravityTermsAnalytical);

    std::string msg = "";
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(gravityTermsFiniteDifferences, gravityTermsAnalytical, 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);
  }

  void testZeroVelocityNonlinearEffects() {
    this->initWithRandomState();
    auto& robotModel = *this->getModelPtr();

    auto state = robotModel.getState();

    state.setZeroVelocities();
    robotModel.setState(state, true, true);

    Eigen::VectorXd nonlinearEffectsAnalytical = Eigen::VectorXd::Zero(robotModel.getDofCount());
    robotModel.getNonlinearEffects(nonlinearEffectsAnalytical);

    Eigen::VectorXd gravityTermsAnalytical = Eigen::VectorXd::Zero(robotModel.getDofCount());
    robotModel.getGravityTerms(gravityTermsAnalytical);

    std::string msg = "";
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(nonlinearEffectsAnalytical, gravityTermsAnalytical, 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);
  }

  void testCoriolisAndCentrifugalTerms() {
    this->initWithRandomState();
    auto& robotModel = *this->getModelPtr();

    Eigen::VectorXd coriolisAndCentrifugalTermsFiniteDifferences = Eigen::VectorXd::Zero(robotModel.getDofCount());
    romo::finite_differences::estimateCoriolisAndCentrifugalTerms(robotModel, coriolisAndCentrifugalTermsFiniteDifferences);

    Eigen::VectorXd coriolisAndCentrifugalTermsAnalytical =
        (robotModel.getNonlinearEffects() - robotModel.getGravityTerms());

    std::string msg = "";
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(coriolisAndCentrifugalTermsFiniteDifferences, coriolisAndCentrifugalTermsAnalytical, 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);
  }

};


TYPED_TEST_CASE(DynamicsWorldToPointTests, FIXTURE_TEST_TYPE);

TYPED_TEST(DynamicsWorldToPointTests, testMassInertiaMatrix) {
  this->testMassMatrix();
}

TYPED_TEST(DynamicsWorldToPointTests, testGravityTerms) {
  this->testGravityTerms();
}

TYPED_TEST(DynamicsWorldToPointTests, testNonlinearEffectsAtZeroVelocity) {
  this->testZeroVelocityNonlinearEffects();
}

TYPED_TEST(DynamicsWorldToPointTests, testCoriolisAndCentrifugalTerms) {
  this->testCoriolisAndCentrifugalTerms();
}

}

