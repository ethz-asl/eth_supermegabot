/*
 * JacobianWorldToPoint_tests.cpp
 *
 *  Created on: Jun 25, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// romo
#include "romo/FiniteDifferences.hpp"

#ifndef ROMO_TEST_JAC_ZERO_TOL
#define ROMO_TEST_JAC_ZERO_TOL 1.0e-7
#endif

namespace romo_test {

template<typename RobotModelFixture_>
class JacobianWorldToPointTests : virtual public RobotModelFixture_ {
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


  void testCalcTranslationJacobianWorldToBody() {
    this->initWithRandomState();
    constexpr unsigned int nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto& body : this->getModelPtr()->getBodyContainer()) {
      {
        Eigen::MatrixXd jacobianFiniteDifference = Eigen::MatrixXd::Zero(3, nU);
        romo::finite_differences::estimateTranslationalJacobianWorldToBody(
            *this->getModelPtr(), body->getBranchEnum(), body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD,
            jacobianFiniteDifference);

        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        this->getModelPtr()->getJacobianTranslationWorldToBody(
            jacobianAnalytic, body->getBranchEnum(), body->getBodyNodeEnum(),
            RobotModelType::CoordinateFrameEnum::WORLD);

        std::string msg{"CalcTranslationJacobianWorldToBodyInWorldFrame for body: " + body->getName()};

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifference, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianFiniteDifference = Eigen::MatrixXd::Zero(3, nU);
        romo::finite_differences::estimateTranslationalJacobianWorldToBody(
            *this->getModelPtr(), body->getBranchEnum(), body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE,
            jacobianFiniteDifference);

        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        this->getModelPtr()->getJacobianTranslationWorldToBody(
            jacobianAnalytic, body->getBranchEnum(), body->getBodyNodeEnum(),
            RobotModelType::CoordinateFrameEnum::BASE);

        std::string msg{"CalcTranslationJacobianWorldToBodyInBaseFrame for body: " + body->getName()};

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifference, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }
    }
  }

  void testCalcRotationJacobianWorldToBody() {
    this->initWithRandomState();
    constexpr unsigned int nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto body : this->getModelPtr()->getBodyContainer()) {
      {
        Eigen::MatrixXd jacobianFiniteDifference = Eigen::MatrixXd::Zero(3, nU);
        romo::finite_differences::estimateRotationalJacobianWorldToBody(
            *this->getModelPtr(), body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD, jacobianFiniteDifference);

        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        this->getModelPtr()->getJacobianRotationWorldToBody(
            jacobianAnalytic, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD);

        std::string msg = "";
        msg.append("CalcRotationJacobianWorldToPointInWorldFrame: ");
        msg.append(body->getName());

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifference, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianFiniteDifference = Eigen::MatrixXd::Zero(3, nU);
        romo::finite_differences::estimateRotationalJacobianWorldToBody(
            *this->getModelPtr(), body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE, jacobianFiniteDifference);

        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        this->getModelPtr()->getJacobianRotationWorldToBody(
            jacobianAnalytic, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE);

        std::string msg = "";
        msg.append("CalcRotationJacobianWorldToPointInBaseFrame: ");
        msg.append(body->getName());

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifference, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }
    }
  }

  void testCalcSpatialJacobianWorldToBody() {
    this->initWithRandomState();
    constexpr unsigned int nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto body : this->getModelPtr()->getBodyContainer()) {
      {
        Eigen::MatrixXd jacobianTranslationalFiniteDifference = Eigen::MatrixXd::Zero(3, nU);
        romo::finite_differences::estimateTranslationalJacobianWorldToBody(
            *this->getModelPtr(), body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD, jacobianTranslationalFiniteDifference);
        Eigen::MatrixXd jacobianRotationalFiniteDifference = Eigen::MatrixXd::Zero(3, nU);
        romo::finite_differences::estimateRotationalJacobianWorldToBody(
            *this->getModelPtr(), body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD, jacobianRotationalFiniteDifference);
        Eigen::MatrixXd jacobianSpatialFiniteDifference = Eigen::MatrixXd::Zero(6, nU);
        jacobianSpatialFiniteDifference.topRows(3) = jacobianRotationalFiniteDifference;
        jacobianSpatialFiniteDifference.bottomRows(3) = jacobianTranslationalFiniteDifference;

        Eigen::MatrixXd jacobianSpatialAnalytic = Eigen::MatrixXd::Zero(6, nU);
        this->getModelPtr()->getJacobianSpatialWorldToBody(
            jacobianSpatialAnalytic, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD);

        std::string msg = "";
        msg.append("CalcSpatialJacobianWorldToBodyInWorldFrame: ");
        msg.append(body->getName());

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianSpatialAnalytic, jacobianSpatialFiniteDifference, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianTranslationalFiniteDifference = Eigen::MatrixXd::Zero(3, nU);
        romo::finite_differences::estimateTranslationalJacobianWorldToBody(
            *this->getModelPtr(), body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE, jacobianTranslationalFiniteDifference);
        Eigen::MatrixXd jacobianRotationalFiniteDifference = Eigen::MatrixXd::Zero(3, nU);
        romo::finite_differences::estimateRotationalJacobianWorldToBody(
            *this->getModelPtr(), body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE, jacobianRotationalFiniteDifference);
        Eigen::MatrixXd jacobianSpatialFiniteDifference = Eigen::MatrixXd::Zero(6, nU);
        jacobianSpatialFiniteDifference.topRows(3) = jacobianRotationalFiniteDifference;
        jacobianSpatialFiniteDifference.bottomRows(3) = jacobianTranslationalFiniteDifference;

        Eigen::MatrixXd jacobianSpatialAnalytic = Eigen::MatrixXd::Zero(6, nU);
        this->getModelPtr()->getJacobianSpatialWorldToBody(
            jacobianSpatialAnalytic, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE);

        std::string msg = "";
        msg.append("CalcSpatialJacobianWorldToBodyInBaseFrame: ");
        msg.append(body->getName());

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianSpatialAnalytic, jacobianSpatialFiniteDifference, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }
    }
  }

  void testGetTranslationJacobianWorldToPointOnBody() {
    // Get a reference to the model.
    auto& model = *this->getModelPtr();
    this->initWithRandomState();

    for (const auto& body : model.getBodyContainer()) {
      {
        // Get the jacobian from world to body->
        Eigen::MatrixXd jacobianWorldToBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianTranslationWorldToBody(
            jacobianWorldToBody, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD);

        // Get the jacobian from world to a position on the body-> The position vector is zero.
        Eigen::MatrixXd jacobianWorldToPointOnBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianTranslationWorldToPointOnBody(
            jacobianWorldToPointOnBody, Eigen::Vector3d::Zero(),
            body->getBranchEnum(), body->getBodyNodeEnum(),
            RobotModelType::CoordinateFrameEnum::WORLD);

        std::string msg = "";
        msg.append("GetTranslationJacobianWorldToPointOnBodyInWorldFrame: ");
        msg.append(body->getName());
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianWorldToBody, jacobianWorldToPointOnBody, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        // Get the jacobian from world to body->
        Eigen::MatrixXd jacobianWorldToBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianTranslationWorldToBody(
            jacobianWorldToBody, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE);

        // Get the jacobian from world to a position on the body-> The position vector is zero.
        Eigen::MatrixXd jacobianWorldToPointOnBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianTranslationWorldToPointOnBody(
            jacobianWorldToPointOnBody, Eigen::Vector3d::Zero(),
            body->getBranchEnum(), body->getBodyNodeEnum(),
            RobotModelType::CoordinateFrameEnum::BASE);

        std::string msg = "";
        msg.append("GetTranslationJacobianWorldToPointOnBodyInBaseFrame: ");
        msg.append(body->getName());
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianWorldToBody, jacobianWorldToPointOnBody, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }
    }
  }

  void testGetRotationJacobianWorldToPointOnBody() {
    // Get a reference to the model.
    auto& model = *this->getModelPtr();
    this->initWithRandomState();

    for (const auto& body : model.getBodyContainer()) {
      {
        // Get the jacobian from world to body->
        Eigen::MatrixXd jacobianWorldToBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianRotationWorldToBody(
            jacobianWorldToBody, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD);

        // Get the jacobian from world to a position on the body-> The position vector is zero.
        Eigen::MatrixXd jacobianWorldToPointOnBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianRotationWorldToPointOnBody(
            jacobianWorldToPointOnBody, Eigen::Vector3d::Zero(),
            body->getBranchEnum(), body->getBodyNodeEnum(),
            RobotModelType::CoordinateFrameEnum::WORLD);

        std::string msg = "";
        msg.append("GetRotationJacobianWorldToPointOnBodyInWorldFrame: ");
        msg.append(body->getName());
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianWorldToBody, jacobianWorldToPointOnBody, 1.0, msg, 1e-8);
      }

      {
        // Get the jacobian from world to body->
        Eigen::MatrixXd jacobianWorldToBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianRotationWorldToBody(
            jacobianWorldToBody, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE);

        // Get the jacobian from world to a position on the body-> The position vector is zero.
        Eigen::MatrixXd jacobianWorldToPointOnBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianRotationWorldToPointOnBody(
            jacobianWorldToPointOnBody, Eigen::Vector3d::Zero(),
            body->getBranchEnum(), body->getBodyNodeEnum(),
            RobotModelType::CoordinateFrameEnum::BASE);

        std::string msg = "";
        msg.append("GetRotationJacobianWorldToPointOnBodyInBaseFrame: ");
        msg.append(body->getName());
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianWorldToBody, jacobianWorldToPointOnBody, 1.0, msg, 1e-8);
      }
    }
  }

  void testGetTranslationJacobianTimeDerivativeFloatingBaseColumns() {
    /* We test the mathematical consistency of the time derivative of the translational Jacobian world to body->
     * The Jacobian that maps generalized velocities to Cartesian space body linear velocities in world frame is given by:
     *
     *  I_JP_b = [I_3x3    S(-I_r_Bb)*C_IB     I_JP_qj]
     *
     *  The time of the I_JP_F will be:
     *
     *  I_dJPdt_b = [0_3x3    S(-I_v_Bb)*C_IB + S(-I_r_Bb)*S(I_w_IB)*C_IB    I_dJPdt_qj]
     *            = [0_3x3    -( S(I_v_Bb) + S(I_r_Bb)*S(I_w_IB) )*C_IB      I_dJPdt_qj]
     */

    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();

    for (const auto& body : robotModel.getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      Eigen::MatrixXd jacobianTranslationTimeDerivativeWorldToBodyInWorldFrameComputed = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
      robotModel.getJacobianTranslationTimeDerivativeWorldToBody(
          jacobianTranslationTimeDerivativeWorldToBodyInWorldFrameComputed,
          branch, node, RobotModelType::CoordinateFrameEnum::WORLD);
      Eigen::MatrixXd jacobianTranslationTimeDerivativeFloatingBaseComputed = jacobianTranslationTimeDerivativeWorldToBodyInWorldFrameComputed.leftCols(6);

      // We test the first six columns of the jacobian.
      Eigen::MatrixXd jacobianTranslationTimeDerivativeWorldToBodyInWorldFrameExpected = Eigen::MatrixXd::Zero(3, 6);
      jacobianTranslationTimeDerivativeWorldToBodyInWorldFrameExpected.leftCols(3) = Eigen::MatrixXd::Zero(3, 3);

      const romo::LinearVelocity linearVelocityBaseToBodyInWorldFrame(
          robotModel.getLinearVelocityWorldToBody(branch, node, RobotModelType::CoordinateFrameEnum::WORLD)
        - robotModel.getState().getLinearVelocityBaseInWorldFrame().toImplementation()
      );

      const romo::Position positionBaseToBodyInWorldFrame(
          robotModel.getPositionBodyToBody(RobotModelType::BodyEnum::BASE,
                                           branch, node, RobotModelType::CoordinateFrameEnum::WORLD)
      );

      const romo::RotationMatrix orientationBaseToWorld(robotModel.getOrientationWorldToBody(RobotModelType::BodyEnum::BASE).transpose());
      const romo::LocalAngularVelocity angularVelocityBaseInWorldFrame = orientationBaseToWorld.rotate(robotModel.getState().getAngularVelocityBaseInBaseFrame());

      const Eigen::MatrixXd expectedJacobianTimeDerivativeRotationalBlock =
          - (kindr::getSkewMatrixFromVector(linearVelocityBaseToBodyInWorldFrame.toImplementation())
             + kindr::getSkewMatrixFromVector(positionBaseToBodyInWorldFrame.toImplementation())
             * kindr::getSkewMatrixFromVector(angularVelocityBaseInWorldFrame.toImplementation()))
          * orientationBaseToWorld.matrix();
      jacobianTranslationTimeDerivativeWorldToBodyInWorldFrameExpected.block<3,3>(0, 3) = expectedJacobianTimeDerivativeRotationalBlock;

      std::string msg = "";
      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianTranslationTimeDerivativeFloatingBaseComputed,
                                   jacobianTranslationTimeDerivativeWorldToBodyInWorldFrameExpected,
                                   1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
    }
  }

  void testGetRotationJacobianTimeDerivativeFloatingBaseColumns() {
    /* We test the mathematical consistency of the time derivative of the rotational Jacobian world to body->
     * The Jacobian that maps generalized velocities to Cartesian space foot angular velocities in world frame is given by:
     *
     *  I_JR_b = [0_3x3    C_IB     I_JR_qj]
     *
     *  The time derivative of I_JR_b will be:
     *
     *  I_dJRdt_b = [0_3x3    S(I_w_IB)*C_IB    I_dJRdt_qj]
     */

    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();

    for (const auto& body : robotModel.getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      // Compute the time derivative of the rotation jacobian.
      Eigen::MatrixXd jacobianRotationTimeDerivativeWorldToBodyInWorldFrameComputed = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
      robotModel.getJacobianRotationTimeDerivativeWorldToBody(
          jacobianRotationTimeDerivativeWorldToBodyInWorldFrameComputed,
          branch, node, RobotModelType::CoordinateFrameEnum::WORLD);
      Eigen::MatrixXd jacobianRotationTimeDerivativeFloatingBaseComputed = jacobianRotationTimeDerivativeWorldToBodyInWorldFrameComputed.leftCols(6);

      // Build the expected time derivative of the rotation jacobian.
      // We test the first six columns of the jacobian.
      Eigen::MatrixXd jacobianRotationTimeDerivativeWorldToBodyInWorldFrameExpected = Eigen::MatrixXd::Zero(3, 6);
      jacobianRotationTimeDerivativeWorldToBodyInWorldFrameExpected.leftCols(3) = Eigen::MatrixXd::Zero(3, 3);

      const romo::RotationMatrix orientationBaseToWorld(robotModel.getOrientationWorldToBody(RobotModelType::BodyEnum::BASE).transpose());
      const romo::LocalAngularVelocity angularVelocityBaseInWorldFrame = orientationBaseToWorld.rotate(robotModel.getState().getAngularVelocityBaseInBaseFrame());

      const Eigen::MatrixXd expectedJacobianTimeDerivativeRotationalBlock =
          kindr::getSkewMatrixFromVector(angularVelocityBaseInWorldFrame.toImplementation())
        * orientationBaseToWorld.toImplementation();
      jacobianRotationTimeDerivativeWorldToBodyInWorldFrameExpected.block<3,3>(0, 3) = expectedJacobianTimeDerivativeRotationalBlock;

      std::string msg = "";
      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianRotationTimeDerivativeFloatingBaseComputed,
                                   jacobianRotationTimeDerivativeWorldToBodyInWorldFrameExpected,
                                   1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
    }
  }

  void testCalcTranslationalJacobianWorldToWholeBodyCom() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();
    constexpr auto nU = RobotStateType::getNumberOfGeneralizedVelocities();

    {
      Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
      Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

      romo::finite_differences::estimateTranslationalJacobianWorldToWholeBodyCom(
          robotModel, RobotModelType::CoordinateFrameEnum::WORLD, jacobianFiniteDifferences);

      robotModel.getJacobianTranslationWorldToCom(
          jacobianAnalytic, RobotModelType::CoordinateFrameEnum::WORLD);

      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferences, 1.0, "computeWorldToCoMJacobianInWorldFrame", ROMO_TEST_JAC_ZERO_TOL);
    }

    {
      Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
      Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

      romo::finite_differences::estimateTranslationalJacobianWorldToWholeBodyCom(
          robotModel, RobotModelType::CoordinateFrameEnum::BASE, jacobianFiniteDifferences);

      robotModel.getJacobianTranslationWorldToCom(
          jacobianAnalytic, RobotModelType::CoordinateFrameEnum::BASE);

      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferences, 1.0, "computeWorldToCoMJacobianInBaseFrame", ROMO_TEST_JAC_ZERO_TOL);
    }
  }

  void testCalcTranslationalJacobianTimeDerivativeWorldToWholeBodyCom() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();
    constexpr auto nU = RobotStateType::getNumberOfGeneralizedVelocities();

    {
      Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
      Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

      romo::finite_differences::estimateTranslationalJacobianTimeDerivativeWorldToWholeBodyComTimeBased(
          robotModel, RobotModelType::CoordinateFrameEnum::WORLD, jacobianFiniteDifferences);

      robotModel.getJacobianTranslationTimeDerivativeWorldToCom(
          jacobianAnalytic, RobotModelType::CoordinateFrameEnum::WORLD);

      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferences, 1.0, "computeWorldToCoMJacobianTimeDerivativeInWorldFrame", ROMO_TEST_JAC_ZERO_TOL);
    }

    {
      Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
      Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

      romo::finite_differences::estimateTranslationalJacobianTimeDerivativeWorldToWholeBodyComTimeBased(
          robotModel, RobotModelType::CoordinateFrameEnum::BASE, jacobianFiniteDifferences);

      robotModel.getJacobianTranslationTimeDerivativeWorldToCom(
          jacobianAnalytic, RobotModelType::CoordinateFrameEnum::BASE);

      KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferences, 1.0, "computeWorldToCoMJacobianTimeDerivativeInBaseFrame", ROMO_TEST_JAC_ZERO_TOL);
    }
  }

  void testCalcTranslationalJacobianTimeDerivativeWorldToBodyInWorldFrame() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();
    constexpr auto nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto& body : robotModel.getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferencesTimeBased = Eigen::MatrixXd::Zero(3, nU);

        romo::finite_differences::estimateTranslationalJacobianTimeDerivativeWorldToBodyInWorldFrame(
            robotModel, branch, node, jacobianFiniteDifferences);

        robotModel.getJacobianTranslationTimeDerivativeWorldToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::WORLD);

        romo::finite_differences::estimateTranslationalJacobianTimeDerivativeWorldToBodyTimeBased(
                  robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD, jacobianFiniteDifferencesTimeBased);

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferences, 1.0,
                                     "computeTranslationalJacobianTimeDerivativeWorldToBodyInWorldFrame", ROMO_TEST_JAC_ZERO_TOL);
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferencesTimeBased, 1.0,
                                     "computeTranslationalJacobianTimeDerivativeWorldToBodyInWorldFrameTimeBased", ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferencesTimeBased = Eigen::MatrixXd::Zero(3, nU);

        robotModel.getJacobianTranslationTimeDerivativeWorldToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::BASE);

        romo::finite_differences::estimateTranslationalJacobianTimeDerivativeWorldToBodyTimeBased(
                  robotModel, branch, node, RobotModelType::CoordinateFrameEnum::BASE, jacobianFiniteDifferencesTimeBased);

        std::string msg = "computeTranslationalJacobianTimeDerivativeWorldToBodyInBaseFrameTimeBased for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferencesTimeBased, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }
    }
  }

  void testCalcTranslationalJacobianTimeDerivativeWorldToPointOnBodyInWorldFrame() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();
    constexpr auto nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto& body : robotModel.getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferencesTimeBased = Eigen::MatrixXd::Zero(3, nU);

        romo::finite_differences::estimateTranslationalJacobianTimeDerivativeWorldToPointOnBodyInWorldFrame(
            robotModel, Eigen::Vector3d::Ones(), branch, node, jacobianFiniteDifferences);

        robotModel.getJacobianTranslationTimeDerivativeWorldToPointOnBody(
            jacobianAnalytic, Eigen::Vector3d::Ones(), branch, node, RobotModelType::CoordinateFrameEnum::WORLD);

        romo::finite_differences::estimateTranslationalJacobianTimeDerivativeWorldToPointOnBodyTimeBased(
                  robotModel, Eigen::Vector3d::Ones(), branch, node, RobotModelType::CoordinateFrameEnum::WORLD, jacobianFiniteDifferencesTimeBased);

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferences, 1.0,
                                     "computeTranslationalJacobianTimeDerivativeWorldToPontOnBodyInWorldFrame", ROMO_TEST_JAC_ZERO_TOL);
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferencesTimeBased, 1.0,
                                     "computeTranslationalJacobianTimeDerivativeWorldToPontOnBodyInWorldFrameTimeBased", ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferencesTimeBased = Eigen::MatrixXd::Zero(3, nU);

        robotModel.getJacobianTranslationTimeDerivativeWorldToPointOnBody(
            jacobianAnalytic, Eigen::Vector3d::Ones(), branch, node, RobotModelType::CoordinateFrameEnum::BASE);

        romo::finite_differences::estimateTranslationalJacobianTimeDerivativeWorldToPointOnBodyTimeBased(
                  robotModel, Eigen::Vector3d::Ones(), branch, node, RobotModelType::CoordinateFrameEnum::BASE, jacobianFiniteDifferencesTimeBased);

        std::string msg = "computeTranslationalJacobianTimeDerivativeWorldToPontOnBodyInBaseFrameTimeBased for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferencesTimeBased, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }
    }
  }

  void testCalcRotationalJacobianTimeDerivativeWorldToBodyInWorldFrame() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();
    constexpr auto nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto& body : robotModel.getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferencesTimeBased = Eigen::MatrixXd::Zero(3, nU);

        romo::finite_differences::estimateRotationalJacobianTimeDerivativeWorldToBodyInWorldFrame(
            robotModel, branch, node, jacobianFiniteDifferences);
        robotModel.getJacobianRotationTimeDerivativeWorldToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::WORLD);
        romo::finite_differences::estimateRotationalJacobianTimeDerivativeWorldToBodyTimeBased(
                  robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD, jacobianFiniteDifferencesTimeBased);

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferences, 1.0,
                                     "computeRotationalJacobianTimeDerivativeWorldToBodyInWorldFrame", ROMO_TEST_JAC_ZERO_TOL);
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferencesTimeBased, 1.0,
                                     "computeRotationalJacobianTimeDerivativeWorldToBodyInWorldFrameTimeBased", ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferencesTimeBased = Eigen::MatrixXd::Zero(3, nU);

        robotModel.getJacobianRotationTimeDerivativeWorldToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::BASE);
        romo::finite_differences::estimateRotationalJacobianTimeDerivativeWorldToBodyTimeBased(
                  robotModel, branch, node, RobotModelType::CoordinateFrameEnum::BASE, jacobianFiniteDifferencesTimeBased);

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferencesTimeBased, 1.0,
                                     "computeRotationalJacobianTimeDerivativeWorldToBodyInBaseFrameTimeBased", ROMO_TEST_JAC_ZERO_TOL);
      }
    }
  }

  void testCalcSpatialJacobianTimeDerivativeWorldToBody() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();
    constexpr auto nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto& body : robotModel.getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(6, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(6, nU);
        Eigen::MatrixXd jacobianFiniteDifferencesTimeBased = Eigen::MatrixXd::Zero(6, nU);

        romo::finite_differences::estimateSpatialJacobianTimeDerivativeWorldToBodyInWorldFrame(
            robotModel, branch, node, jacobianFiniteDifferences);

        robotModel.getJacobianSpatialTimeDerivativeWorldToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::WORLD);

        romo::finite_differences::estimateSpatialJacobianTimeDerivativeWorldToBodyTimeBased(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD, jacobianFiniteDifferencesTimeBased);

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferences, 1.0,
                                     "computeSpatialJacobianTimeDerivativeWorldToBodyInWorldFrame", ROMO_TEST_JAC_ZERO_TOL);
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferencesTimeBased, 1.0,
                                     "computeSpatialJacobianTimeDerivativeWorldToBodyInWorldFrameTimeBased", ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(6, nU);
        Eigen::MatrixXd jacobianFiniteDifferencesTimeBased = Eigen::MatrixXd::Zero(6, nU);

        robotModel.getJacobianSpatialTimeDerivativeWorldToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::BASE);

        romo::finite_differences::estimateSpatialJacobianTimeDerivativeWorldToBodyTimeBased(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::BASE, jacobianFiniteDifferencesTimeBased);

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianAnalytic, jacobianFiniteDifferencesTimeBased, 1.0,
                                     "computeSpatialJacobianTimeDerivativeWorldToBodyInBaseFrameTimeBased", ROMO_TEST_JAC_ZERO_TOL);
      }
    }
  }

  void testCalcTranslationalJacobianFloatingBaseToBody() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();
    constexpr auto nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto& body : robotModel.getBodyContainer()) {
      if (body->getIsFixedBody()) {
        continue;
      }

      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

        romo::finite_differences::estimateTranslationalJacobianFloatingBaseToPointOnBody(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD, Eigen::Vector3d::Zero(), jacobianFiniteDifferences);

        robotModel.getJacobianTranslationFloatingBaseToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::WORLD);

        const std::string msg = "testCalcTranslationalJacobianFloatingBaseToBodyInWorldFrame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(
            jacobianAnalytic, jacobianFiniteDifferences, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

        romo::finite_differences::estimateTranslationalJacobianFloatingBaseToPointOnBody(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::BASE, Eigen::Vector3d::Zero(), jacobianFiniteDifferences);

        robotModel.getJacobianTranslationFloatingBaseToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::BASE);

        const std::string msg = "testCalcTranslationalJacobianFloatingBaseToBodyInBaseFrame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(
            jacobianAnalytic, jacobianFiniteDifferences, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

        // Point should be in body frame.
        romo::finite_differences::estimateTranslationalJacobianFloatingBaseToPointOnBody(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD,
            body->getPositionBodyToBodyCom(RobotModelType::CoordinateFrameEnum::BODY), jacobianFiniteDifferences);

        robotModel.getJacobianTranslationFloatingBaseToBodyCom(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::WORLD);

        const std::string msg = "testCalcTranslationalJacobianFloatingBaseToBodyComInWorldFrame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(
            jacobianAnalytic, jacobianFiniteDifferences, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

        romo::finite_differences::estimateTranslationalJacobianFloatingBaseToPointOnBody(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::BASE,
            body->getPositionBodyToBodyCom(RobotModelType::CoordinateFrameEnum::BODY), jacobianFiniteDifferences);

        robotModel.getJacobianTranslationFloatingBaseToBodyCom(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::BASE);

        const std::string msg = "testCalcTranslationalJacobianFloatingBaseToBodyInBaseFrame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(
            jacobianAnalytic, jacobianFiniteDifferences, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

    }
  }

  void testCalcTranslationalJacobianFloatingBaseToPointOnBody() {
    // Get a reference to the model.
    auto& model = *this->getModelPtr();
    this->initWithRandomState();

    for (const auto& body : model.getBodyContainer()) {
      if (body->getIsFixedBody()) {
        continue;
      }

      {
        // Get the jacobian from floating base to body->
        Eigen::MatrixXd jacobianFloatingBaseToBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianTranslationFloatingBaseToBody(
            jacobianFloatingBaseToBody, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD);

        // Get the jacobian from floating base to a position on the body-> The position vector is zero.
        Eigen::MatrixXd jacobianFloatingBaseToPointOnBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianTranslationFloatingBaseToPointOnBody(
            jacobianFloatingBaseToPointOnBody, Eigen::Vector3d::Zero(),
            body->getBranchEnum(), body->getBodyNodeEnum(),
            RobotModelType::CoordinateFrameEnum::WORLD);

        std::string msg = "";
        msg.append("GetTranslationalJacobianFloatingBaseToPointOnBodyInWorldFrame: ");
        msg.append(body->getName());
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianFloatingBaseToBody, jacobianFloatingBaseToPointOnBody, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        // Get the jacobian from floating base to body->
        Eigen::MatrixXd jacobianFloatingBaseToBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianTranslationFloatingBaseToBody(
            jacobianFloatingBaseToBody, body->getBranchEnum(),
            body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE);

        // Get the jacobian from floating base to a position on the body-> The position vector is zero.
        Eigen::MatrixXd jacobianFloatingBaseToPointOnBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
        model.getJacobianTranslationFloatingBaseToPointOnBody(
            jacobianFloatingBaseToPointOnBody, Eigen::Vector3d::Zero(),
            body->getBranchEnum(), body->getBodyNodeEnum(),
            RobotModelType::CoordinateFrameEnum::BASE);

        std::string msg = "";
        msg.append("GetTranslationalJacobianFloatingBaseToPointOnBodyInBaseFrame: ");
        msg.append(body->getName());
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianFloatingBaseToBody, jacobianFloatingBaseToPointOnBody, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, model.getDofCount());
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, model.getDofCount());
        Eigen::Vector3d point(1.0, 1.0, 1.0);

        romo::finite_differences::estimateTranslationalJacobianFloatingBaseToPointOnBody(
            model, body->getBranchEnum(), body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD, point, jacobianFiniteDifferences);

        model.getJacobianTranslationFloatingBaseToPointOnBody(
            jacobianAnalytic, point, body->getBranchEnum(), body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD);

        const std::string msg = "testCalcTranslationalJacobianFloatingBaseToPointOnBodyInWorldFrame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(
            jacobianAnalytic, jacobianFiniteDifferences, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, model.getDofCount());
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, model.getDofCount());
        Eigen::Vector3d point(1.0, 1.0, 1.0);

        romo::finite_differences::estimateTranslationalJacobianFloatingBaseToPointOnBody(
            model, body->getBranchEnum(), body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE, point, jacobianFiniteDifferences);

        model.getJacobianTranslationFloatingBaseToPointOnBody(
            jacobianAnalytic, point, body->getBranchEnum(), body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE);

        const std::string msg = "testCalcTranslationalJacobianFloatingBaseToPointOnBodyInBaseFrame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(
            jacobianAnalytic, jacobianFiniteDifferences, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }
    }

  }

  void testCalcRotationalJacobianFloatingBaseToBody() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();
    constexpr auto nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto& body : robotModel.getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node   = body->getBodyNodeEnum();

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

        romo::finite_differences::estimateRotationalJacobianFloatingBaseToBody(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::WORLD, jacobianFiniteDifferences);

        robotModel.getJacobianRotationFloatingBaseToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::WORLD);

        const std::string msg = "testCalcRotationalJacobianFloatingBaseToBodyInWorldFrame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(
            jacobianAnalytic, jacobianFiniteDifferences, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

      {
        Eigen::MatrixXd jacobianAnalytic = Eigen::MatrixXd::Zero(3, nU);
        Eigen::MatrixXd jacobianFiniteDifferences = Eigen::MatrixXd::Zero(3, nU);

        romo::finite_differences::estimateRotationalJacobianFloatingBaseToBody(
            robotModel, branch, node, RobotModelType::CoordinateFrameEnum::BASE, jacobianFiniteDifferences);

        robotModel.getJacobianRotationFloatingBaseToBody(
            jacobianAnalytic, branch, node, RobotModelType::CoordinateFrameEnum::BASE);

        const std::string msg = "testCalcRotationJacobianFloatingBaseToBodyInBaseFrame for body: " + body->getName();
        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(
            jacobianAnalytic, jacobianFiniteDifferences, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
      }

    }
  }

  void testCalcRotationalJacobianFloatingBaseToPointOnBody() {
      // Get a reference to the model.
      auto& model = *this->getModelPtr();
      this->initWithRandomState();

      for (const auto& body : model.getBodyContainer()) {
        if (body->getIsFixedBody()) {
          continue;
        }

        {
          // Get the jacobian from floating base to body->
          Eigen::MatrixXd jacobianFloatingBaseToBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
          model.getJacobianRotationFloatingBaseToBody(
              jacobianFloatingBaseToBody, body->getBranchEnum(),
              body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::WORLD);

          // Get the jacobian from floating base to a position on the body-> The position vector is zero.
          Eigen::MatrixXd jacobianFloatingBaseToPointOnBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
          model.getJacobianRotationFloatingBaseToPointOnBody(
              jacobianFloatingBaseToPointOnBody, Eigen::Vector3d::Zero(),
              body->getBranchEnum(), body->getBodyNodeEnum(),
              RobotModelType::CoordinateFrameEnum::WORLD);

          std::string msg = "";
          msg.append("GetRotationalJacobianFloatingBaseToPointOnBodyInWorldFrame: ");
          msg.append(body->getName());
          KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianFloatingBaseToBody, jacobianFloatingBaseToPointOnBody, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
        }

        {
          // Get the jacobian from floating base to body->
          Eigen::MatrixXd jacobianFloatingBaseToBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
          model.getJacobianRotationFloatingBaseToBody(
              jacobianFloatingBaseToBody, body->getBranchEnum(),
              body->getBodyNodeEnum(), RobotModelType::CoordinateFrameEnum::BASE);

          // Get the jacobian from floating base to a position on the body-> The position vector is zero.
          Eigen::MatrixXd jacobianFloatingBaseToPointOnBody = Eigen::MatrixXd::Zero(3, model.getDofCount());
          model.getJacobianRotationFloatingBaseToPointOnBody(
              jacobianFloatingBaseToPointOnBody, Eigen::Vector3d::Zero(),
              body->getBranchEnum(), body->getBodyNodeEnum(),
              RobotModelType::CoordinateFrameEnum::BASE);

          std::string msg = "";
          msg.append("GetRotationalJacobianFloatingBaseToPointOnBodyInBaseFrame: ");
          msg.append(body->getName());
          KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianFloatingBaseToBody, jacobianFloatingBaseToPointOnBody, 1.0, msg, ROMO_TEST_JAC_ZERO_TOL);
        }

      }

    }

  void testCalcManipulabilityGradient() {
    // Initialize the model with a random state.
    this->initWithRandomState();

    // Get a reference to the model.
    auto& robotModel = *this->getModelPtr();
    constexpr auto nU = RobotStateType::getNumberOfGeneralizedVelocities();

    for (const auto& body : robotModel.getBodyContainer()) {
      const auto branch = body->getBranchEnum();
      const auto node = body->getBodyNodeEnum();

      {
        Eigen::VectorXd manipulabilityGradientFiniteDifferences = Eigen::VectorXd::Zero(nU);
        romo::finite_differences::estimateManipulabilityGradientWorldToBody(
            robotModel, branch, node,
            manipulabilityGradientFiniteDifferences);

        Eigen::VectorXd manipulabilityGradientAnalytical = Eigen::VectorXd::Zero(nU);
        robotModel.getManipulabilityMeasureGradientWorldToBody(
            manipulabilityGradientAnalytical, branch, node);

        KINDR_ASSERT_DOUBLE_MX_EQ_ZT(manipulabilityGradientAnalytical, manipulabilityGradientFiniteDifferences, 1.0,
                                     "manipulabilityGradient", ROMO_TEST_JAC_ZERO_TOL);
      }


    }

  }

};


TYPED_TEST_CASE(JacobianWorldToPointTests, FIXTURE_TEST_TYPE);

TYPED_TEST(JacobianWorldToPointTests, CalcTranslationJacobianWorldToBody) {
  this->testCalcTranslationJacobianWorldToBody();
}

TYPED_TEST(JacobianWorldToPointTests, CalcRotationJacobianWorldToBody) {
  this->testCalcRotationJacobianWorldToBody();
}

TYPED_TEST(JacobianWorldToPointTests, CalcSpatialJacobianWorldToBody) {
  this->testCalcSpatialJacobianWorldToBody();
}

TYPED_TEST(JacobianWorldToPointTests, GetTransationJacobianWorldToPointOnBody) {
  this->testGetTranslationJacobianWorldToPointOnBody();
}

TYPED_TEST(JacobianWorldToPointTests, GetRotationJacobianWorldToPointOnBody) {
  this->testGetRotationJacobianWorldToPointOnBody();
}

#ifndef ROMO_TEST_FIXED_BASE
TYPED_TEST(JacobianWorldToPointTests, CalcTranslationJacobianFloatingBaseToBody) {
  this->testCalcTranslationalJacobianFloatingBaseToBody();
}

TYPED_TEST(JacobianWorldToPointTests, CalcTranslationJacobianFloatingBaseToPoinOnBody) {
  this->testCalcTranslationalJacobianFloatingBaseToPointOnBody();
}

TYPED_TEST(JacobianWorldToPointTests, CalcRotationJacobianFloatingBaseToBody) {
  this->testCalcRotationalJacobianFloatingBaseToBody();
}

TYPED_TEST(JacobianWorldToPointTests, CalcRotationJacobianFloatingBaseToPointOnBody) {
  this->testCalcRotationalJacobianFloatingBaseToPointOnBody();
}

TYPED_TEST(JacobianWorldToPointTests, GetTranslationJacobianTimeDerivativeFloatingBaseColumns) {
  this->testGetTranslationJacobianTimeDerivativeFloatingBaseColumns();
}

TYPED_TEST(JacobianWorldToPointTests, GetRotationJacobianTimeDerivativeFloatingBaseColumns) {
  this->testGetRotationJacobianTimeDerivativeFloatingBaseColumns();
}
#endif

TYPED_TEST(JacobianWorldToPointTests, CalcTranslationalJacobianWorldToWholeBodyCom) {
  this->testCalcTranslationalJacobianWorldToWholeBodyCom();
}

TYPED_TEST(JacobianWorldToPointTests, CalcTranslationalJacobianTimeDerivativeWorldToWholeBodyCom) {
  this->testCalcTranslationalJacobianTimeDerivativeWorldToWholeBodyCom();
}

TYPED_TEST(JacobianWorldToPointTests, CalcTranslationalJacobianTimeDerivativeWorldToBodyInWorldFrame) {
  this->testCalcTranslationalJacobianTimeDerivativeWorldToBodyInWorldFrame();
}

TYPED_TEST(JacobianWorldToPointTests, CalcTranslationalJacobianTimeDerivativeWorldToPointOnBodyInWorldFrame) {
  this->testCalcTranslationalJacobianTimeDerivativeWorldToPointOnBodyInWorldFrame();
}

TYPED_TEST(JacobianWorldToPointTests, CalcRotationalJacobianTimeDerivativeWorldToBodyInWorldFrame) {
  this->testCalcRotationalJacobianTimeDerivativeWorldToBodyInWorldFrame();
}

TYPED_TEST(JacobianWorldToPointTests, CalcSpatialJacobianTimeDerivativeWorldToBody) {
  this->testCalcSpatialJacobianTimeDerivativeWorldToBody();
}

TYPED_TEST(JacobianWorldToPointTests, CalcManipulabilityGradient) {
  this->testCalcManipulabilityGradient();
}


}
