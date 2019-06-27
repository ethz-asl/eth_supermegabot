/*
 * RobotModelRbdl_implementation_jacobian.tpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Dario Bellicoso
 */

#include "romo_rbdl/rbdl_kinematics.hpp"
#include "romo_rbdl/RobotModelRbdl.hpp"

namespace romo_rbdl {


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianSpatialWorldToBody(
  Eigen::MatrixXd& jacobian,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {
  assert(jacobian.rows() == RD::getNumSpatialDof() && jacobian.cols() == getDofCount());
  return getJacobianSpatialWorldToPointOnBody(jacobian, Eigen::Vector3d::Zero(), branch, node, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianSpatialWorldToPointOnBody(
  Eigen::MatrixXd& jacobian,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {

  assert(jacobian.rows() == 6 && jacobian.cols() == getDofCount());

  RigidBodyDynamics::CalcSpatialJacobianWorldToPointInWorldFrame(*rbdlModel_,
                                                                 stateGeneralizedPositionsQuaternionRBDL_,
                                                                 this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId(),
                                                                 positionBodyToPointOnBodyInBodyFrame,
                                                                 jacobian,
                                                                 false);

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      const Eigen::Matrix3d& rotationWorldToBase = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody();
      jacobian.topRows(3) = rotationWorldToBase * jacobian.topRows(3);
      jacobian.bottomRows(3) = rotationWorldToBase * jacobian.bottomRows(3);
    } break;

    default: {
      MELO_ERROR_STREAM("[RobotModel::getJacobianSpatialWorldToBody] Unhandled coordinate frame.");
    }
  }

  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianSpatialTimeDerivativeWorldToBody(
  Eigen::MatrixXd& jacobianSpatialTimeDerivativeWorldToBody,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const
{
  return getJacobianSpatialTimeDerivativeWorldToPointOnBody(jacobianSpatialTimeDerivativeWorldToBody,
                                                            Eigen::Vector3d::Zero(),
                                                            branch,
                                                            node,
                                                            frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianSpatialTimeDerivativeWorldToBody(
  Eigen::MatrixXd& jacobianTimeDerivativeWorldToBody,
  const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const
{
  return getJacobianSpatialTimeDerivativeWorldToPointOnBody(jacobianTimeDerivativeWorldToBody,
                                                            jacobianSpatialWorldToBodyInWorldFrame,
                                                            Eigen::Vector3d::Zero(),
                                                            branch,
                                                            node,
                                                            frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianSpatialTimeDerivativeWorldToPointOnBody(
  Eigen::MatrixXd& jacobianTimeDerivativeWorldToBody,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {

  // Calculate spatial Jacobian from world to body in world frame
  Eigen::MatrixXd jacobianSpatialWorldToBodyInWorldFrame = Eigen::MatrixXd::Zero(6,getDofCount());
  getJacobianSpatialWorldToBody(jacobianSpatialWorldToBodyInWorldFrame, branch, node, CoordinateFrameEnum::WORLD);

  return getJacobianSpatialTimeDerivativeWorldToPointOnBody(jacobianTimeDerivativeWorldToBody,
                                                            jacobianSpatialWorldToBodyInWorldFrame,
                                                            positionBodyToPointOnBodyInBodyFrame,
                                                            branch,
                                                            node,
                                                            frame );
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianSpatialTimeDerivativeWorldToPointOnBody(
  Eigen::MatrixXd& jacobianTimeDerivativeWorldToBody,
  const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {
  assert(jacobianTimeDerivativeWorldToBody.rows() == 6 && jacobianTimeDerivativeWorldToBody.cols() == getDofCount());
  assert(jacobianSpatialWorldToBodyInWorldFrame.rows() == 6 && jacobianSpatialWorldToBodyInWorldFrame.cols() == getDofCount());

  const int rbdlId = this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId();
  Eigen::MatrixXd hessian(6, getDofCount());
  for (unsigned int jointId = 0; jointId < getDofCount(); jointId++) {
    hessian.setZero();
    CalcHessianSpatialWorldToPointInWorldFrameForState(*rbdlModel_,
                                                       stateGeneralizedPositionsQuaternionRBDL_,
                                                       jointId, rbdlId, positionBodyToPointOnBodyInBodyFrame,
                                                       jacobianSpatialWorldToBodyInWorldFrame, hessian, false);
    jacobianTimeDerivativeWorldToBody += hessian * stateGeneralizedVelocitiesAngularRBDL_[jointId];
  }

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      const Eigen::Vector3d angularVelocityWorldToBaseInBaseFrame = -this->getState()
        .getAngularVelocityBaseInBaseFrame().toImplementation();
      const Eigen::Matrix3d skewAngularVelocityWorldToBaseInBaseFrame(
        kindr::getSkewMatrixFromVector(angularVelocityWorldToBaseInBaseFrame));
      const Eigen::Matrix3d& rotationWorldToBase = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody();

      jacobianTimeDerivativeWorldToBody.bottomRows(3) = skewAngularVelocityWorldToBaseInBaseFrame
        * rotationWorldToBase * jacobianSpatialWorldToBodyInWorldFrame.bottomRows(3)
        + rotationWorldToBase * jacobianTimeDerivativeWorldToBody.bottomRows(3);

      jacobianTimeDerivativeWorldToBody.topRows(3) = skewAngularVelocityWorldToBaseInBaseFrame
        * rotationWorldToBase * jacobianSpatialWorldToBodyInWorldFrame.topRows(3)
        + rotationWorldToBase * jacobianTimeDerivativeWorldToBody.topRows(3);
    } break;

    default: {
      MELO_ERROR_STREAM("[RobotModel::getJacobianSpatialTimeDerivativeWorldToBody] Unhandled coordinate frame.");
    }
  }

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationWorldToBody(
  Eigen::MatrixXd& jacobian,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const
{
  return getJacobianTranslationWorldToPointOnBody(jacobian, Eigen::Vector3d::Zero(), branch, node, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationWorldToPointOnBody(
  Eigen::MatrixXd& jacobian,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {

  assert(jacobian.rows() == 3 && jacobian.cols() == getDofCount());

  int body_id = this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId();

  CalcTranslationalJacobianWorldToPointInWorldFrame(*rbdlModel_,
                                                    stateGeneralizedPositionsQuaternionRBDL_,
                                                    body_id,
                                                    positionBodyToPointOnBodyInBodyFrame,
                                                    jacobian,
                                                    false);

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      jacobian = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody() * jacobian;
    } break;

    default: {
      MELO_ERROR_STREAM("[RobotModel::getJacobianTranslationWorldToPoint] Unhandled coordinate frame.");
    }
  }

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationWorldToBodyCom(
  Eigen::MatrixXd& jacobian,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {
  const Eigen::Vector3d positionBodyToBodyCom = this->bodyBranchNodeContainer_.at(branch).at(node)->getPositionBodyToBodyCom(CoordinateFrameEnum::BODY);
  return getJacobianTranslationWorldToPointOnBody(jacobian, positionBodyToBodyCom, branch, node, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationWorldToCom(
  Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const
{
  assert(jacobian.rows() == 3 && jacobian.cols() == getDofCount());

  RigidBodyDynamics::CalcTranslationalJacobianWorldToCoMInWorldFrame(
    *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, jacobian, false);

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      jacobian = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody() * jacobian;
    } break;

    default: {
      MELO_ERROR_STREAM("[RobotModel::getJacobianTranslationWorldToCom] Unhandled coordinate frame.");
    }
  }

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationTimeDerivativeWorldToBody(
  Eigen::MatrixXd& jacobianTranslationTimeDerivative,
  const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {

  return getJacobianTranslationTimeDerivativeWorldToPointOnBody(jacobianTranslationTimeDerivative,
                                                                jacobianSpatialWorldToBodyInWorldFrame,
                                                                Eigen::Vector3d::Zero(),
                                                                branch, node, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationTimeDerivativeWorldToBody(
  Eigen::MatrixXd& jacobianTranslationTimeDerivative,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const
{
  return getJacobianTranslationTimeDerivativeWorldToPointOnBody(jacobianTranslationTimeDerivative,
                                                                Eigen::Vector3d::Zero(),
                                                                branch, node, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationTimeDerivativeWorldToPointOnBody(
  Eigen::MatrixXd& jacobianTranslationTimeDerivative,
  const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {
  assert(jacobianTranslationTimeDerivative.rows() == 3 && jacobianTranslationTimeDerivative.cols() == getDofCount());

  const int rbdlId = this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId();
  Eigen::MatrixXd hessianTranslationWorldToPointInWorldFrame(3, getDofCount());
  for (unsigned int jointId = 0; jointId < getDofCount(); jointId++) {
    hessianTranslationWorldToPointInWorldFrame.setZero();
    CalcTranslationalHessianWorldToPointInWorldFrameForState(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, jointId, rbdlId,
      positionBodyToPointOnBodyInBodyFrame, jacobianSpatialWorldToBodyInWorldFrame,
      hessianTranslationWorldToPointInWorldFrame, false);

    jacobianTranslationTimeDerivative += hessianTranslationWorldToPointInWorldFrame
      * stateGeneralizedVelocitiesAngularRBDL_[jointId];
  }

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      const Eigen::Vector3d angularVelocityInertiaInBaseFrame = -this->getState()
        .getAngularVelocityBaseInBaseFrame().toImplementation();
      const Eigen::Matrix3d skewAngularVelocityInertiaInBaseFrame(
        kindr::getSkewMatrixFromVector(angularVelocityInertiaInBaseFrame));
      const Eigen::Matrix3d& rotationWorldToBase = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody();

      jacobianTranslationTimeDerivative = skewAngularVelocityInertiaInBaseFrame
        * rotationWorldToBase * jacobianSpatialWorldToBodyInWorldFrame.bottomRows(3)
        + rotationWorldToBase * jacobianTranslationTimeDerivative;
    } break;

    default: {
      MELO_ERROR_STREAM("[RobotModel::getJacobianTranslationTimeDerivativeWorldToBody] Unhandled coordinate frame.");
    }
  }

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationTimeDerivativeWorldToPointOnBody(
    Eigen::MatrixXd& jacobianTranslationTimeDerivative,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const
{
  Eigen::MatrixXd jacobianSpatialWorldToBodyInWorldFrame = Eigen::MatrixXd::Zero(6,getDofCount());
  getJacobianSpatialWorldToPointOnBody(jacobianSpatialWorldToBodyInWorldFrame, positionBodyToPointOnBodyInBodyFrame, branch, node, CoordinateFrameEnum::WORLD);

  return getJacobianTranslationTimeDerivativeWorldToPointOnBody(jacobianTranslationTimeDerivative,
                                                                jacobianSpatialWorldToBodyInWorldFrame,
                                                                positionBodyToPointOnBodyInBodyFrame,
                                                                branch,
                                                                node,
                                                                frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationTimeDerivativeWorldToCom(
  Eigen::MatrixXd& jacobianTranslationTimeDerivativeWorldToCom,
  CoordinateFrameEnum frame) const {
  assert(jacobianTranslationTimeDerivativeWorldToCom.rows() == 3
           && jacobianTranslationTimeDerivativeWorldToCom.cols() == getDofCount());

  // Calculate spatial Jacobian from world to body in world frame
  Eigen::MatrixXd jacobianTranslationWorldToComInWorldFrame = Eigen::MatrixXd::Zero(3,getDofCount());
  getJacobianTranslationWorldToCom(jacobianTranslationWorldToComInWorldFrame,
                                   CoordinateFrameEnum::WORLD);

  // compute I_dJ
  Eigen::MatrixXd hessian(3, getDofCount());
  for (unsigned int jointId = 0; jointId < getDofCount(); ++jointId) {
    hessian.setZero();
    RigidBodyDynamics::CalcTranslationalHessianWorldToCoMInWorldFrameForState(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
      jointId, hessian, false);

    jacobianTranslationTimeDerivativeWorldToCom += hessian * stateGeneralizedVelocitiesAngularRBDL_[jointId];
  }

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      const Eigen::Vector3d angularVelocityInertiaInBaseFrame = -this->getState()
        .getAngularVelocityBaseInBaseFrame().toImplementation();
      const Eigen::Matrix3d skewAngularVelocityInertiaInBaseFrame(
        kindr::getSkewMatrixFromVector(angularVelocityInertiaInBaseFrame));
      const Eigen::Matrix3d& rotationWorldToBase = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody();

      jacobianTranslationTimeDerivativeWorldToCom = skewAngularVelocityInertiaInBaseFrame
        * rotationWorldToBase * jacobianTranslationWorldToComInWorldFrame
        + rotationWorldToBase * jacobianTranslationTimeDerivativeWorldToCom;
    } break;

    default: {
      MELO_ERROR_STREAM(
        "[RobotModel::getJacobianTranslationTimeDerivativeWorldToCom] Unhandled coordinate frame.");
    }
  }

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianRotationWorldToBody(
  Eigen::MatrixXd& jacobian,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {
  return getJacobianRotationWorldToPointOnBody(jacobian, Eigen::Vector3d::Zero(), branch, node, frame);
}

template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianRotationWorldToPointOnBody(
  Eigen::MatrixXd& jacobian,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {

  assert(jacobian.rows() == 3 && jacobian.cols() == getDofCount());

  CalcRotationJacobianInWorldFrame(*rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
                                   this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId(),
                                   positionBodyToPointOnBodyInBodyFrame, jacobian, false);

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      jacobian = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody() * jacobian;
    } break;

    default: {
      MELO_ERROR_STREAM("[RobotModel::getJacobianRotationWorldToBody] Unhandled coordinate frame.");
    }
  }

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianRotationTimeDerivativeWorldToBody(
  Eigen::MatrixXd& jacobianRotationTimeDerivative,
  const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const
{
  return getJacobianRotationTimeDerivativeWorldToPointOnBody(jacobianRotationTimeDerivative,
                                                             jacobianSpatialWorldToBodyInWorldFrame,
                                                             Eigen::Vector3d::Zero(),
                                                             branch,
                                                             node,
                                                             frame);
}

template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianRotationTimeDerivativeWorldToBody(
  Eigen::MatrixXd& jacobianRotationTimeDerivative,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const
{
  return getJacobianRotationTimeDerivativeWorldToPointOnBody(jacobianRotationTimeDerivative,
                                                             Eigen::Vector3d::Zero(),
                                                             branch,
                                                             node,
                                                             frame);
}

template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianRotationTimeDerivativeWorldToPointOnBody(
  Eigen::MatrixXd& jacobianRotationTimeDerivative,
  const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {
  assert(jacobianRotationTimeDerivative.rows() == 3 && jacobianRotationTimeDerivative.cols() == getDofCount());

  const int rbdlId = this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId();
  Eigen::MatrixXd hessianRotationWorldToPointInWorldFrame(3, getDofCount());
  for (unsigned int jointId = 0; jointId < getDofCount(); jointId++) {
    hessianRotationWorldToPointInWorldFrame.setZero();
    CalcRotationalHessianWorldToPointInWorldFrameForState(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, jointId, rbdlId,
      positionBodyToPointOnBodyInBodyFrame, jacobianSpatialWorldToBodyInWorldFrame,
      hessianRotationWorldToPointInWorldFrame, false);

    jacobianRotationTimeDerivative += hessianRotationWorldToPointInWorldFrame
      * stateGeneralizedVelocitiesAngularRBDL_[jointId];
  }

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      const Eigen::Vector3d angularVelocityInertiaInBaseFrame = -this->getState()
        .getAngularVelocityBaseInBaseFrame().toImplementation();
      const Eigen::Matrix3d skewAngularVelocityInertiaInBaseFrame(
        kindr::getSkewMatrixFromVector(angularVelocityInertiaInBaseFrame));
      const Eigen::Matrix3d& rotationWorldToBase = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody();

      jacobianRotationTimeDerivative = skewAngularVelocityInertiaInBaseFrame
        * rotationWorldToBase * jacobianSpatialWorldToBodyInWorldFrame.topRows(3)
        + rotationWorldToBase * jacobianRotationTimeDerivative;
    } break;

    default: {
      MELO_ERROR_STREAM("[RobotModel::getJacobianRotationTimeDerivativeWorldToBody] Unhandled coordinate frame.");
    }
  }

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianRotationTimeDerivativeWorldToPointOnBody(
    Eigen::MatrixXd& jacobianRotationTimeDerivative,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const
{
  Eigen::MatrixXd jacobianSpatialWorldToBodyInWorldFrame = Eigen::MatrixXd::Zero(6,getDofCount());
  getJacobianSpatialWorldToPointOnBody(jacobianSpatialWorldToBodyInWorldFrame, positionBodyToPointOnBodyInBodyFrame, branch, node, CoordinateFrameEnum::WORLD);

  return getJacobianRotationTimeDerivativeWorldToPointOnBody(jacobianRotationTimeDerivative,
                                                             jacobianSpatialWorldToBodyInWorldFrame,
                                                             positionBodyToPointOnBodyInBodyFrame,
                                                             branch,
                                                             node,
                                                             frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationFloatingBaseToBody(
    Eigen::MatrixXd& jacobian,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const {

  return getJacobianTranslationFloatingBaseToPointOnBody(jacobian, Eigen::Vector3d::Zero(), branch, node, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationFloatingBaseToPointOnBody(
    Eigen::MatrixXd& jacobian,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const {
  assert((jacobian.rows() == 3) && (jacobian.cols() == getDofCount()));
  RigidBodyDynamics::CalcTranslationalJacobianFloatingBaseToPointOnBodyInWorldFrame(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
      this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId(),
      positionBodyToPointOnBodyInBodyFrame, jacobian, false);
  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      jacobian = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody() * jacobian;
    } break;

    default: {
      MELO_ERROR_STREAM("[RobotModel::getJacobianTranslationFloatingBaseToPointOnBody] Unhandled coordinate frame.");
    }
  }

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianTranslationFloatingBaseToBodyCom(
  Eigen::MatrixXd& jacobian,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const {
  const Eigen::Vector3d positionBodyToBodyCom = this->bodyBranchNodeContainer_.at(branch).at(node)->getPositionBodyToBodyCom(CoordinateFrameEnum::BODY);
  return getJacobianTranslationFloatingBaseToPointOnBody(jacobian, positionBodyToBodyCom, branch, node, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianRotationFloatingBaseToBody(
    Eigen::MatrixXd& jacobian,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const {

  return getJacobianRotationFloatingBaseToPointOnBody(jacobian, Eigen::Vector3d::Zero(), branch, node, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJacobianRotationFloatingBaseToPointOnBody(
    Eigen::MatrixXd& jacobian,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const {
  assert((jacobian.rows() == 3) && (jacobian.cols() == getDofCount()));
  RigidBodyDynamics::CalcRotationalJacobianFloatingBaseToPointOnBodyInWorldFrame(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
      this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId(),
      positionBodyToPointOnBodyInBodyFrame, jacobian, false);
  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      // do nothing
    } break;

    case (CoordinateFrameEnum::BASE): {
      jacobian = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody() * jacobian;
    } break;

    default: {
      MELO_ERROR_STREAM("[RobotModel::getJacobianRotationFloatingBaseToPointOnBody] Unhandled coordinate frame.");
    }
  }

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
double RobotModelRbdl<ConcreteDescription_, RobotState_>::getManipulabilityMeasure(
    BranchEnum branch,
    BodyNodeEnum bodyNode) const {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, getDofCount());
  getJacobianTranslationWorldToBody(jacobian, branch, bodyNode, CoordinateFrameEnum::WORLD);

  return std::sqrt((jacobian*jacobian.transpose()).determinant());
}

template<typename ConcreteDescription_, typename RobotState_>
void RobotModelRbdl<ConcreteDescription_, RobotState_>::getManipulabilityMeasureGradientWorldToBody(
    Eigen::VectorXd& manipulabilityMeasureGradient,
    BranchEnum branch,
    BodyNodeEnum bodyNode) const {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, getDofCount());
  getJacobianTranslationWorldToBody(jacobian, branch, bodyNode, CoordinateFrameEnum::WORLD);

  Eigen::MatrixXd jacobianPinv;
  kindr::pseudoInverse(jacobian, jacobianPinv);

  const double manipulabilityMeasure = std::sqrt((jacobian*jacobian.transpose()).determinant());

  Eigen::MatrixXd hessian(3, getDofCount());
  for (unsigned int i=0; i<getDofCount(); ++i) {
    hessian.setZero();
    getHessianTranslationWorldToBodyForState(hessian, i, branch, bodyNode, CoordinateFrameEnum::WORLD);
    manipulabilityMeasureGradient(i) = manipulabilityMeasure * (hessian*jacobianPinv).trace();
  }
}

template<typename ConcreteDescription_, typename RobotState_>
void RobotModelRbdl<ConcreteDescription_, RobotState_>::getManipulabilityMeasureGradientBodyToBody(
    Eigen::VectorXd& manipulabilityMeasureGradient,
    BranchEnum fromBranch,
    BodyNodeEnum fromBodyNode,
    BranchEnum toBranch,
    BodyNodeEnum toBodyNode) const {
  // TODO!!!
}


} /* namespace romo */

