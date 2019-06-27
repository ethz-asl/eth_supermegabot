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

  void testMassMatrix(bool initialize = true, Eigen::MatrixXd* massInertiaMatrixAnalyticalOut = nullptr) {
    if (initialize){
      this->initWithRandomState();
    }
    auto& robotModel = *this->getModelPtr();

    Eigen::MatrixXd massInertiaMatrixFiniteDifferences =
        Eigen::MatrixXd::Zero(robotModel.getDofCount(), robotModel.getDofCount());
    romo::finite_differences::estimateMassMatrix(robotModel, massInertiaMatrixFiniteDifferences);
    Eigen::MatrixXd massInertiaMatrixAnalytical =
        Eigen::MatrixXd::Zero(robotModel.getDofCount(), robotModel.getDofCount());
    robotModel.getMassInertiaMatrix(massInertiaMatrixAnalytical);

    std::string msg = "";
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(massInertiaMatrixFiniteDifferences, massInertiaMatrixAnalytical, 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);

    if (massInertiaMatrixAnalyticalOut){
      *massInertiaMatrixAnalyticalOut = massInertiaMatrixAnalytical;
    }
  }

  void testGravityTerms(bool initialize = true, Eigen::VectorXd* gravityTermsAnalyticalOut = nullptr) {
    if (initialize){
      this->initWithRandomState();
    }
    auto& robotModel = *this->getModelPtr();

    Eigen::VectorXd gravityTermsFiniteDifferences = Eigen::VectorXd::Zero(robotModel.getDofCount());
    romo::finite_differences::estimateGravityTerms(robotModel, gravityTermsFiniteDifferences);

    Eigen::VectorXd gravityTermsAnalytical = Eigen::VectorXd::Zero(robotModel.getDofCount());
    robotModel.getGravityTerms(gravityTermsAnalytical);

    std::string msg = "";
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(gravityTermsFiniteDifferences, gravityTermsAnalytical, 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);

    if (gravityTermsAnalyticalOut){
      *gravityTermsAnalyticalOut = gravityTermsAnalytical;
    }
  }

  void testZeroVelocityNonlinearEffects(bool initialize = true, Eigen::VectorXd* gravityTermsAnalyticalOut = nullptr) {
    if (initialize){
      this->initWithRandomState();
    }
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

    if (gravityTermsAnalyticalOut){
      *gravityTermsAnalyticalOut = gravityTermsAnalytical;
    }
  }

  void testCoriolisAndCentrifugalTerms(bool initialize = true, Eigen::VectorXd* coriolisAndCentrifugalTermsAnalyticalOut = nullptr) {
    if (initialize){
      this->initWithRandomState();
    }
    auto& robotModel = *this->getModelPtr();

    Eigen::VectorXd coriolisAndCentrifugalTermsFiniteDifferences = Eigen::VectorXd::Zero(robotModel.getDofCount());
    romo::finite_differences::estimateCoriolisAndCentrifugalTerms(robotModel, coriolisAndCentrifugalTermsFiniteDifferences);

    Eigen::VectorXd coriolisAndCentrifugalTermsAnalytical =
        (robotModel.getNonlinearEffects() - robotModel.getGravityTerms());

    std::string msg = "";
    KINDR_ASSERT_DOUBLE_MX_EQ_ZT(coriolisAndCentrifugalTermsFiniteDifferences, coriolisAndCentrifugalTermsAnalytical, 1.0, msg, ROMO_TEST_DYN_ZERO_TOL);

    if (coriolisAndCentrifugalTermsAnalyticalOut){
      *coriolisAndCentrifugalTermsAnalyticalOut = coriolisAndCentrifugalTermsAnalytical;
    }
  }

  void addOneToAllIndividualMasses(){
    auto& robotModel = *this->getModelPtr();
    for (auto bodyKey : RobotModelType::RD::getBodyKeys()){

      BodyEnum bodyEnum = bodyKey.getEnum();
      double newMass = robotModel.getBodyMass(bodyEnum)+1.0;

      EXPECT_TRUE(robotModel.setIndividualBodyMass(bodyEnum, newMass)) << "Setting the individual mass of "<<bodyKey.getName()<<" failed";
    }
  }

  void addOneToAllMasses(){
    auto& robotModel = *this->getModelPtr();
    for (auto bodyKey : RobotModelType::RD::getBodyKeys()){

      BodyEnum bodyEnum = bodyKey.getEnum();
      double newMass = robotModel.getBodyMass(bodyEnum)+1.0;

      if (robotModel.getBody(bodyEnum).getIsFixedBody()){
        EXPECT_FALSE(robotModel.setBodyMass(bodyEnum, newMass)) << "Setting the mass of "<<bodyKey.getName()<<" succeeded, but this is a fixed body ";
      } else {
        EXPECT_TRUE(robotModel.setBodyMass(bodyEnum, newMass)) << "Setting the mass of "<<bodyKey.getName()<<" failed";
      }
    }
  }

  void addOnesToAllIndividualComs(){
    auto& robotModel = *this->getModelPtr();
    for (auto bodyKey : RobotModelType::RD::getBodyKeys()){

      BodyEnum bodyEnum = bodyKey.getEnum();
      Eigen::Vector3d originalCom = Eigen::Vector3d::Zero();
      try{
        originalCom = robotModel.getBody(bodyEnum).getPositionBodyToBodyCom(CoordinateFrameEnum::BODY);
      } catch (const std::runtime_error& e){}
      Eigen::Vector3d newCoM = originalCom+Eigen::Vector3d::Ones();

      EXPECT_TRUE(robotModel.setPositionBodyToIndividualBodyCom(newCoM, bodyEnum, CoordinateFrameEnum::BODY)) << "Setting the individual CoM of "<<bodyKey.getName()<<" failed";
    }
  }

  void addOnesToAllComs(){
    auto& robotModel = *this->getModelPtr();
    for (auto bodyKey : RobotModelType::RD::getBodyKeys()){

      BodyEnum bodyEnum = bodyKey.getEnum();
      Eigen::Vector3d originalCom = Eigen::Vector3d::Zero();
      try{
        originalCom = robotModel.getBody(bodyEnum).getPositionBodyToBodyCom(CoordinateFrameEnum::BODY);
      } catch (const std::runtime_error& e){}
      Eigen::Vector3d newCoM = originalCom+Eigen::Vector3d::Ones();

      if (robotModel.getBody(bodyEnum).getIsFixedBody()){
        EXPECT_FALSE(robotModel.setPositionBodyToBodyCom(newCoM, bodyEnum, CoordinateFrameEnum::BODY)) << "Setting the CoM of "<<bodyKey.getName()<<" succeeded, but this is a fixed body ";
      } else {
        EXPECT_TRUE(robotModel.setPositionBodyToBodyCom(newCoM, bodyEnum, CoordinateFrameEnum::BODY)) << "Setting the CoM of "<<bodyKey.getName()<<" failed";
      }
    }
  }

  void addIdentityToAllIndividualInertias(){
    auto& robotModel = *this->getModelPtr();
    for (auto bodyKey : RobotModelType::RD::getBodyKeys()){

      BodyEnum bodyEnum = bodyKey.getEnum();
      Eigen::Matrix3d originalInertia = Eigen::Matrix3d::Zero();
      try{
        originalInertia = robotModel.getBody(bodyEnum).getInertiaMatrix();
      } catch (const std::runtime_error& e){}
      Eigen::Matrix3d newInertia = originalInertia+Eigen::Matrix3d::Identity();

      EXPECT_TRUE(robotModel.setIndividualBodyInertiaMatrix(bodyEnum, newInertia)) << "Setting the individual inertia of "<<bodyKey.getName()<<" failed";
    }
  }

  void addIdentityToAllInertias(){
    auto& robotModel = *this->getModelPtr();
    for (auto bodyKey : RobotModelType::RD::getBodyKeys()){

      BodyEnum bodyEnum = bodyKey.getEnum();
      Eigen::Matrix3d originalInertia = Eigen::Matrix3d::Zero();
      try{
        originalInertia = robotModel.getBody(bodyEnum).getInertiaMatrix();
      } catch (const std::runtime_error& e){}
      Eigen::Matrix3d newInertia = originalInertia+Eigen::Matrix3d::Identity();

      if (robotModel.getBody(bodyEnum).getIsFixedBody()){
        EXPECT_FALSE(robotModel.setBodyInertiaMatrix(bodyEnum, newInertia)) << "Setting the inertia of "<<bodyKey.getName()<<" succeeded, but this is a fixed body ";
      } else {
        EXPECT_TRUE(robotModel.setBodyInertiaMatrix(bodyEnum, newInertia)) << "Setting the inertia of "<<bodyKey.getName()<<" failed";
      }
    }
  }

  void testAndGetDynamicsTerms(bool initializeTheFixture,
                              Eigen::MatrixXd& massInertiaMatrixAnalyticalOut,
                              Eigen::VectorXd& gravityTermsAnalyticalOut,
                              Eigen::VectorXd& coriolisAndCentrifugalTermsAnalyticalOut){
    this->testMassMatrix(initializeTheFixture, &massInertiaMatrixAnalyticalOut);
    testGravityTerms(false, &gravityTermsAnalyticalOut);
    this->testCoriolisAndCentrifugalTerms(false, &coriolisAndCentrifugalTermsAnalyticalOut);
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


TYPED_TEST(DynamicsWorldToPointTests, testSetIndividualMass) {
  Eigen::MatrixXd oldMassMatrix;
  Eigen::VectorXd oldGravityTerms;
  Eigen::VectorXd oldCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(true, oldMassMatrix, oldGravityTerms, oldCoriolisAndCentrifugalTerms);

  this->addOneToAllIndividualMasses();

  Eigen::MatrixXd newMassMatrix;
  Eigen::VectorXd newGravityTerms;
  Eigen::VectorXd newCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(false, newMassMatrix, newGravityTerms, newCoriolisAndCentrifugalTerms);

  ASSERT_FALSE(oldMassMatrix.isApprox(newMassMatrix))<< "After changing the masses of the bodies, the mass matrix did not change";
  ASSERT_FALSE(oldGravityTerms.isApprox(newGravityTerms))<< "After changing the masses of the bodies, the gravity terms did not change";
  ASSERT_FALSE(oldCoriolisAndCentrifugalTerms.isApprox(newCoriolisAndCentrifugalTerms))<< "After changing the masses of the bodies, the coriolis and centrifugal terms did not change";
}

TYPED_TEST(DynamicsWorldToPointTests, testSetMass) {
  Eigen::MatrixXd oldMassMatrix;
  Eigen::VectorXd oldGravityTerms;
  Eigen::VectorXd oldCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(true, oldMassMatrix, oldGravityTerms, oldCoriolisAndCentrifugalTerms);

  this->addOneToAllMasses();

  Eigen::MatrixXd newMassMatrix;
  Eigen::VectorXd newGravityTerms;
  Eigen::VectorXd newCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(false, newMassMatrix, newGravityTerms, newCoriolisAndCentrifugalTerms);

  ASSERT_FALSE(oldMassMatrix.isApprox(newMassMatrix))<< "After changing the masses of the bodies, the mass matrix did not change";
  ASSERT_FALSE(oldGravityTerms.isApprox(newGravityTerms))<< "After changing the masses of the bodies, the gravity terms did not change";
  ASSERT_FALSE(oldCoriolisAndCentrifugalTerms.isApprox(newCoriolisAndCentrifugalTerms))<< "After changing the masses of the bodies, the coriolis and centrifugal terms did not change";
}

TYPED_TEST(DynamicsWorldToPointTests, testSetIndividualCoM) {
  Eigen::MatrixXd oldMassMatrix;
  Eigen::VectorXd oldGravityTerms;
  Eigen::VectorXd oldCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(true, oldMassMatrix, oldGravityTerms, oldCoriolisAndCentrifugalTerms);

  this->addOnesToAllIndividualComs();

  Eigen::MatrixXd newMassMatrix;
  Eigen::VectorXd newGravityTerms;
  Eigen::VectorXd newCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(false, newMassMatrix, newGravityTerms, newCoriolisAndCentrifugalTerms);

  ASSERT_FALSE(oldMassMatrix.isApprox(newMassMatrix))<< "After changing the CoMs of the bodies, the mass matrix did not change";
  ASSERT_FALSE(oldGravityTerms.isApprox(newGravityTerms))<< "After changing the CoMs of the bodies, the gravity terms did not change";
  ASSERT_FALSE(oldCoriolisAndCentrifugalTerms.isApprox(newCoriolisAndCentrifugalTerms))<< "After changing the CoMs of the bodies, the coriolis and centrifugal terms did not change";
}

TYPED_TEST(DynamicsWorldToPointTests, testSetCoM) {
  Eigen::MatrixXd oldMassMatrix;
  Eigen::VectorXd oldGravityTerms;
  Eigen::VectorXd oldCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(true, oldMassMatrix, oldGravityTerms, oldCoriolisAndCentrifugalTerms);

  this->addOnesToAllComs();

  Eigen::MatrixXd newMassMatrix;
  Eigen::VectorXd newGravityTerms;
  Eigen::VectorXd newCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(false, newMassMatrix, newGravityTerms, newCoriolisAndCentrifugalTerms);

  ASSERT_FALSE(oldMassMatrix.isApprox(newMassMatrix))<< "After changing the CoMs of the bodies, the mass matrix did not change";
  ASSERT_FALSE(oldGravityTerms.isApprox(newGravityTerms))<< "After changing the CoMs of the bodies, the gravity terms did not change";
  ASSERT_FALSE(oldCoriolisAndCentrifugalTerms.isApprox(newCoriolisAndCentrifugalTerms))<< "After changing the CoMs of the bodies, the coriolis and centrifugal terms did not change";
}

TYPED_TEST(DynamicsWorldToPointTests, testSetIndividualInertia) {
  Eigen::MatrixXd oldMassMatrix;
  Eigen::VectorXd oldGravityTerms;
  Eigen::VectorXd oldCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(true, oldMassMatrix, oldGravityTerms, oldCoriolisAndCentrifugalTerms);

  this->addIdentityToAllIndividualInertias();

  Eigen::MatrixXd newMassMatrix;
  Eigen::VectorXd newGravityTerms;
  Eigen::VectorXd newCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(false, newMassMatrix, newGravityTerms, newCoriolisAndCentrifugalTerms);

  ASSERT_FALSE(oldMassMatrix.isApprox(newMassMatrix))<< "After changing the inertias of the bodies, the mass matrix did not change";
  ASSERT_TRUE(oldGravityTerms.isApprox(newGravityTerms))<< "After changing the inertias of the bodies, the gravity terms did change (they should not have)";
  ASSERT_FALSE(oldCoriolisAndCentrifugalTerms.isApprox(newCoriolisAndCentrifugalTerms))<< "After changing the inertias of the bodies, the coriolis and centrifugal terms did not change";
}

TYPED_TEST(DynamicsWorldToPointTests, testSetInertia) {
  Eigen::MatrixXd oldMassMatrix;
  Eigen::VectorXd oldGravityTerms;
  Eigen::VectorXd oldCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(true, oldMassMatrix, oldGravityTerms, oldCoriolisAndCentrifugalTerms);

  this->addIdentityToAllInertias();

  Eigen::MatrixXd newMassMatrix;
  Eigen::VectorXd newGravityTerms;
  Eigen::VectorXd newCoriolisAndCentrifugalTerms;
  this->testAndGetDynamicsTerms(false, newMassMatrix, newGravityTerms, newCoriolisAndCentrifugalTerms);

  ASSERT_FALSE(oldMassMatrix.isApprox(newMassMatrix))<< "After changing the inertias of the bodies, the mass matrix did not change";
  ASSERT_TRUE(oldGravityTerms.isApprox(newGravityTerms))<< "After changing the inertias of the bodies, the gravity terms did change (they should not have)";
  ASSERT_FALSE(oldCoriolisAndCentrifugalTerms.isApprox(newCoriolisAndCentrifugalTerms))<< "After changing the inertias of the bodies, the coriolis and centrifugal terms did not change";
}

}
