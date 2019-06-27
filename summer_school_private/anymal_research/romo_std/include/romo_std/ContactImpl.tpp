/*
 * ContactImpl.tpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Christian Gehring, Dario Bellicoso
 */

#include "romo_std/ContactImpl.hpp"

namespace romo_std {

template<typename ConcreteDescription_, typename RobotState_>
ContactImpl<ConcreteDescription_, RobotState_>::ContactImpl(ContactImpl<ConcreteDescription_, RobotState_>::RobotModelPtr model, ContactEnum contactEnum)
    : model_(model),
      forceInWorldFrame_(),
      torqueInWorldFrame_(),
      normalInWorldFrame_(romo::Vector::UnitZ()),
      positionMovableParentToContactInMovableParentFrame_(),
      orientationMovableParentToContact_(),
      state_(ContactState::OPEN),
      contactEnum_(contactEnum),
      branchEnum_(RD::template mapEnums<BranchEnum>(contactEnum)),
      bodyNodeEnum_(RD::template mapEnums<BodyNodeEnum>(contactEnum))
{

}

template<typename ConcreteDescription_, typename RobotState_>
romo::Force ContactImpl<ConcreteDescription_, RobotState_>::getForce(CoordinateFrameEnum frame) const
{
  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error("[ContactImpl::getForce] Frame is not supported!");
  }
  return forceInWorldFrame_;
}


template<typename ConcreteDescription_, typename RobotState_>
void ContactImpl<ConcreteDescription_, RobotState_>::setForce(const romo::Force& force, CoordinateFrameEnum frame)
{
  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error("[ContactImpl::setForce] Frame is not supported!");
  }
  forceInWorldFrame_ = force;
}


template<typename ConcreteDescription_, typename RobotState_>
romo::Torque ContactImpl<ConcreteDescription_, RobotState_>::getTorque(CoordinateFrameEnum frame) const
{
  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error("[ContactImpl::getTorque] Frame is not supported!");
  }
  return torqueInWorldFrame_;
}


template<typename ConcreteDescription_, typename RobotState_>
void ContactImpl<ConcreteDescription_, RobotState_>::setTorque(const romo::Torque& torque, CoordinateFrameEnum frame)
{
  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error("[ContactImpl::setTorque] Frame is not supported!");
  }
  torqueInWorldFrame_ = torque;
}


template<typename ConcreteDescription_, typename RobotState_>
romo::Vector ContactImpl<ConcreteDescription_, RobotState_>::getNormal(CoordinateFrameEnum frame) const
{
  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error("[ContactImpl::getNormal] Frame is not supported!");
  }
  return normalInWorldFrame_;
}


template<typename ConcreteDescription_, typename RobotState_>
void ContactImpl<ConcreteDescription_, RobotState_>::setNormal(const romo::Vector& normal, CoordinateFrameEnum frame)
{
  if (frame != CoordinateFrameEnum::WORLD) {
    throw std::runtime_error("[ContactImpl::setNormal] Frame is not supported!");
  }
  normalInWorldFrame_ = normal;
}


template<typename ConcreteDescription_, typename RobotState_>
const typename ContactImpl<ConcreteDescription_, RobotState_>::ContactState& ContactImpl<ConcreteDescription_, RobotState_>::getState() const
{
  return state_;
}


template<typename ConcreteDescription_, typename RobotState_>
void ContactImpl<ConcreteDescription_, RobotState_>::setState(ContactState state)
{
  state_ = state;
}


template<typename ConcreteDescription_, typename RobotState_>
const romo::Position& ContactImpl<ConcreteDescription_, RobotState_>::getPositionMovableParentToContactInMovableParentFrame() const {
  return positionMovableParentToContactInMovableParentFrame_;
}


template<typename ConcreteDescription_, typename RobotState_>
void ContactImpl<ConcreteDescription_, RobotState_>::setPositionMovableParentToContactInMovableParentFrame(romo::Position position) {
  positionMovableParentToContactInMovableParentFrame_ = position;
}


template<typename ConcreteDescription_, typename RobotState_>
const romo::RotationQuaternion& ContactImpl<ConcreteDescription_, RobotState_>::getOrientationMovableParentToContact() const {
  return orientationMovableParentToContact_;
}


template<typename ConcreteDescription_, typename RobotState_>
void ContactImpl<ConcreteDescription_, RobotState_>::setOrientationMovableParentToContact(romo::RotationQuaternion orientation) {
  orientationMovableParentToContact_ = orientation;
}


template<typename ConcreteDescription_, typename RobotState_>
romo::Position ContactImpl<ConcreteDescription_, RobotState_>::getPositionWorldToContact(CoordinateFrameEnum frame) const {
  return romo::Position(model_->getBody(branchEnum_, bodyNodeEnum_).getPositionWorldToPointOnBody(
    positionMovableParentToContactInMovableParentFrame_.toImplementation(), frame));
}


template<typename ConcreteDescription_, typename RobotState_>
romo::RotationQuaternion ContactImpl<ConcreteDescription_, RobotState_>::getOrientationWorldToContact() const {
  return ( orientationMovableParentToContact_ * romo::RotationMatrix(model_->getBody(branchEnum_, bodyNodeEnum_).getOrientationWorldToBody()) );
}


template<typename ConcreteDescription_, typename RobotState_>
bool ContactImpl<ConcreteDescription_, RobotState_>::getJacobianSpatialWorldToContact( Eigen::MatrixXd& jacobian,
                                                                               CoordinateFrameEnum frame) const {
  return model_->getJacobianSpatialWorldToPointOnBody(jacobian, positionMovableParentToContactInMovableParentFrame_.toImplementation(),
                                                      branchEnum_, bodyNodeEnum_, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool ContactImpl<ConcreteDescription_, RobotState_>::getJacobianSpatialTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian,
                                                                                             CoordinateFrameEnum frame) const {
  return model_->getJacobianSpatialTimeDerivativeWorldToPointOnBody(jacobian, positionMovableParentToContactInMovableParentFrame_.toImplementation(),
                                                                    branchEnum_, bodyNodeEnum_, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool ContactImpl<ConcreteDescription_, RobotState_>::getJacobianSpatialTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobianDerivative,
                                                                                             const Eigen::MatrixXd& jacobian,
                                                                                             CoordinateFrameEnum frame) const {
  return model_->getJacobianSpatialTimeDerivativeWorldToPointOnBody(jacobianDerivative, jacobian,
                                                                    positionMovableParentToContactInMovableParentFrame_.toImplementation(),
                                                                    branchEnum_, bodyNodeEnum_, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool ContactImpl<ConcreteDescription_, RobotState_>::getJacobianTranslationWorldToContact( Eigen::MatrixXd& jacobian,
                                                                                   CoordinateFrameEnum frame) const {
  return model_->getJacobianTranslationWorldToPointOnBody(jacobian, positionMovableParentToContactInMovableParentFrame_.toImplementation(),
                                                          branchEnum_, bodyNodeEnum_, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool ContactImpl<ConcreteDescription_, RobotState_>::getJacobianTranslationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian,
                                                                                                 CoordinateFrameEnum frame) const {
  return model_->getJacobianTranslationTimeDerivativeWorldToPointOnBody(jacobian, positionMovableParentToContactInMovableParentFrame_.toImplementation(),
                                                                        branchEnum_, bodyNodeEnum_, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool ContactImpl<ConcreteDescription_, RobotState_>::getJacobianTranslationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian,
                                                                                                 const Eigen::MatrixXd& spatialJacobian,
                                                                                                 CoordinateFrameEnum frame) const {
  return model_->getJacobianTranslationTimeDerivativeWorldToPointOnBody(jacobian, spatialJacobian,
                                                                        positionMovableParentToContactInMovableParentFrame_.toImplementation(),
                                                                        branchEnum_, bodyNodeEnum_, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool ContactImpl<ConcreteDescription_, RobotState_>::getJacobianRotationWorldToContact( Eigen::MatrixXd& jacobian,
                                                                                CoordinateFrameEnum frame) const {
  return model_->getJacobianRotationWorldToPointOnBody(jacobian, positionMovableParentToContactInMovableParentFrame_.toImplementation(),
                                                       branchEnum_, bodyNodeEnum_, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool ContactImpl<ConcreteDescription_, RobotState_>::getJacobianRotationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian,
                                                                                              CoordinateFrameEnum frame) const {
  return model_->getJacobianRotationTimeDerivativeWorldToPointOnBody(jacobian, positionMovableParentToContactInMovableParentFrame_.toImplementation(),
                                                                     branchEnum_, bodyNodeEnum_, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool ContactImpl<ConcreteDescription_, RobotState_>::getJacobianRotationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian,
                                                                                              const Eigen::MatrixXd& spatialJacobianInWorldFrame,
                                                                                              CoordinateFrameEnum frame) const {
  return model_->getJacobianRotationTimeDerivativeWorldToPointOnBody(jacobian, spatialJacobianInWorldFrame,
                                                                     positionMovableParentToContactInMovableParentFrame_.toImplementation(),
                                                                     branchEnum_, bodyNodeEnum_, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
romo::LinearVelocity ContactImpl<ConcreteDescription_, RobotState_>::getLinearVelocityWorldToContact( CoordinateFrameEnum frame) const {
  return romo::LinearVelocity( model_->getLinearVelocityWorldToPointOnBody(branchEnum_, bodyNodeEnum_,
                                                     positionMovableParentToContactInMovableParentFrame_.toImplementation(), frame) );
}


template<typename ConcreteDescription_, typename RobotState_>
romo::LocalAngularVelocity ContactImpl<ConcreteDescription_, RobotState_>::getAngularVelocityWorldToContact( CoordinateFrameEnum frame) const {
  return romo::LocalAngularVelocity( model_->getAngularVelocityWorldToBody(branchEnum_, bodyNodeEnum_, frame) );
}

} /* namespace romo */
