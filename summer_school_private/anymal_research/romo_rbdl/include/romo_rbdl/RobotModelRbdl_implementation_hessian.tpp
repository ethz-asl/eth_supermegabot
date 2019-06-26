/*
 * RobotModelRbdl_implementation_hessian.tpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Dario Bellicoso
 */

#include "romo_rbdl/rbdl_kinematics.hpp"
#include "romo_rbdl/RobotModelRbdl.hpp"

namespace romo_rbdl {


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getHessianTranslationWorldToBodyForState(
    Eigen::MatrixXd& hessian,
    unsigned int qIndex,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const
{
  assert(hessian.rows() == 3 && hessian.cols() == getDofCount());

  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error(
        "[RobotModel::getHessianTranslationWorldToBodyForState] Frame is not supported!");
  }

  return RigidBodyDynamics::CalcTranslationalHessianWorldToPointInWorldFrameForState(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, qIndex,
      this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId(), Eigen::Vector3d::Zero(), hessian,
      false);
}

template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getHessianTranslationWorldToBodyForState(
    Eigen::MatrixXd& hessian,
    unsigned int qIndex,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame,
    Eigen::MatrixXd& spatialJacobian) const {
  assert(hessian.rows() == 3 && hessian.cols() == getDofCount());

  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error(
        "[RobotModel::getHessianTranslationWorldToBodyForState] Frame is not supported!");
  }

  return RigidBodyDynamics::CalcTranslationalHessianWorldToPointInWorldFrameForState(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, qIndex,
      this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId(), Eigen::Vector3d::Zero(), hessian,
      spatialJacobian,
      false);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getHessianRotationWorldToBodyForState(
    Eigen::MatrixXd& hessian,
    unsigned int qIndex,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const
{

  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error(
        "[RobotModel::getHessianRotationWorldToBodyForState] Frame is not supported!");
  }

  return RigidBodyDynamics::CalcRotationalHessianWorldToPointInWorldFrameForState(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, qIndex,
      this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId(), Eigen::Vector3d::Zero(), hessian,
      false);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getHessianSpatialWorldToBodyForState(
    Eigen::MatrixXd& hessian,
    unsigned int qIndex,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const
{

  assert(hessian.rows() == 6 && hessian.cols() == getDofCount());

  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error("[RobotModel::getJacobianTranslationWorldToBody] Frame is not supported!");
  }

  CalcHessianSpatialWorldToPointInWorldFrameForState(*rbdlModel_,
                                                stateGeneralizedPositionsQuaternionRBDL_, qIndex,
                                                this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId(),
                                                Eigen::Vector3d::Zero(), hessian,
                                                false);

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getHessianTranslationWorldToComForState(
    Eigen::MatrixXd& hessian,
    unsigned int qIndex,
    CoordinateFrameEnum frame) const
{
  assert(hessian.rows() == 3 && hessian.cols() == getDofCount());

  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error("[RobotModel::getJacobianTranslationWorldToBody] Frame is not supported!");
  }

  return RigidBodyDynamics::CalcTranslationalHessianWorldToCoMInWorldFrameForState(*rbdlModel_,
                                                                      stateGeneralizedPositionsQuaternionRBDL_,
                                                                      qIndex,
                                                                      hessian,
                                                                      false);
}


} /* namespace romo */

