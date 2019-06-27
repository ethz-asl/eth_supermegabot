/*
 * RobotModelRbdl_implementation_dynamics.tpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Dario Bellicoso
 */

#include <romo/common/RigidBody.hpp>
#include "romo_rbdl/rbdl_kinematics.hpp"
#include "romo_rbdl/RobotModelRbdl.hpp"

namespace romo_rbdl {


template<typename ConcreteDescription_, typename RobotState_>
double RobotModelRbdl<ConcreteDescription_, RobotState_>::getGravityAcceleration() const {
  return fabs(rbdlModel_->gravity.z());
}


template<typename ConcreteDescription_, typename RobotState_>
const Eigen::Vector3d& RobotModelRbdl<ConcreteDescription_, RobotState_>::getGravityVectorInWorldFrame() const {
  return rbdlModel_->gravity;
}


template<typename ConcreteDescription_, typename RobotState_>
double RobotModelRbdl<ConcreteDescription_, RobotState_>::getBodyMass(BodyEnum body) const {
  return this->bodyContainer_[body]->getMass();
}


template<typename ConcreteDescription_, typename RobotState_>
double RobotModelRbdl<ConcreteDescription_, RobotState_>::getBodyMass(BranchEnum branch,
                                                                      BodyNodeEnum node) const {
  return this->bodyBranchNodeContainer_.at(branch).at(node)->getMass();
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::setBodyMass(BodyEnum body, const double mass, bool updateBodyInertia) const {
  const std::shared_ptr<romo::RigidBody<ConcreteDescription_>>& bodyPtr = this->bodyContainer_[body];
  if (bodyPtr->getIsFixedBody()){
    MELO_ERROR_STREAM("[RobotModel::setBodyMass] You cannot use setBodyMass on a fixed body (called on body "<< bodyPtr->getName()<<"); use setIndividualBodyMass instead" );
    return false;
  } else {
    unsigned int bodyId = bodyPtr->getBodyId();
    rbdlModel_->mBodies[bodyId]->AbsorbFixedChildren(false);
    rbdlModel_->mBodies[bodyId]->SetIndividualMass(mass, updateBodyInertia);
    return true;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::setIndividualBodyMass(BodyEnum body, double mass, bool updateBodyInertia) const{
  const std::shared_ptr<romo::RigidBody<ConcreteDescription_>>& bodyPtr = this->bodyContainer_[body];
  if (bodyPtr->getIsFixedBody()){
    unsigned int bodyId = bodyPtr->getBodyId() - rbdlModel_->fixed_body_discriminator;
    if (rbdlModel_->mFixedBodies[bodyId]->GetIsNulled()){
      MELO_WARN_THROTTLE_STREAM(1.0, "[RobotModel::setIndividualBodyMass] The body "<<bodyPtr->getName()<<" is a fixed body that was nulled (you called a setBodyXXXX function in the past), but you are setting it's individual mass.");
    }
    rbdlModel_->mFixedBodies[bodyId]->SetIndividualMass(mass, updateBodyInertia);
    return true;
  } else {
    unsigned int bodyId = bodyPtr->getBodyId();
    if (rbdlModel_->mBodies[bodyId]->GetChildrenWereAbsorbed()){
      MELO_WARN_THROTTLE_STREAM(1.0, "[RobotModel::setIndividualBodyMass] The body "<<bodyPtr->getName()<<" absorbed its children (you called a setBodyXXXX function in the past), but you are setting it's individual mass, instead of setBodyMass.");
    }
    rbdlModel_->mBodies[bodyId]->SetIndividualMass(mass, updateBodyInertia);
    return true;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
double RobotModelRbdl<ConcreteDescription_, RobotState_>::getRootMass() const {
  return getBodyMass(BodyEnum::BASE);
}


template<typename ConcreteDescription_, typename RobotState_>
double RobotModelRbdl<ConcreteDescription_, RobotState_>::getLimbMass(BranchEnum branch) const {
  double limbMass = 0.0;
  const auto& container = this->bodyBranchNodeContainer_.at(branch);
  for (const auto& body : container | boost::adaptors::map_values) {
    if (body->getIsFixedBody()) {
      continue;
    }
    limbMass += body->getMass();
  }
  return limbMass;
}


template<typename ConcreteDescription_, typename RobotState_>
double RobotModelRbdl<ConcreteDescription_, RobotState_>::getTotalMass() const {
  double totalMass = 0.0;
  for (const auto& body : this->bodyContainer_) {
    totalMass += body->getMass();
  }
  return totalMass;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::setBodyInertiaProperties(BodyEnum body, const double mass, const Eigen::Vector3d& centerOfMassInBodyFrame, const Eigen::Matrix3d& inertiaAtCom, bool updateBodyInertia) const{
  const std::shared_ptr<romo::RigidBody<ConcreteDescription_>>& bodyPtr = this->bodyContainer_[body];
  if (bodyPtr->getIsFixedBody()){
    MELO_ERROR_STREAM("[RobotModel::setBodyInertiaProperties] You cannot use setBodyInertiaProperties on a fixed body (called on body "<< bodyPtr->getName()<<"); use setIndividualBodyInertiaProperties instead" );
    return false;
  } else {
    unsigned int bodyId = bodyPtr->getBodyId();
    rbdlModel_->mBodies[bodyId]->AbsorbFixedChildren(false);
    rbdlModel_->mBodies[bodyId]->SetIndividualInertialProperties(mass, centerOfMassInBodyFrame,inertiaAtCom, updateBodyInertia);
    return true;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::setIndividualBodyInertiaProperties(BodyEnum body, const double mass, const Eigen::Vector3d& centerOfMassInBodyFrame, const Eigen::Matrix3d& inertiaAtCom, bool updateBodyInertia ) const{
  const std::shared_ptr<romo::RigidBody<ConcreteDescription_>>& bodyPtr = this->bodyContainer_[body];
  if (bodyPtr->getIsFixedBody()){
    unsigned int bodyId = bodyPtr->getBodyId() - rbdlModel_->fixed_body_discriminator;
    if (rbdlModel_->mFixedBodies[bodyId]->GetIsNulled()){
      MELO_WARN_THROTTLE_STREAM(1.0, "[RobotModel::setIndividualBodyInertiaProperties] The body "<<bodyPtr->getName()<<" is a fixed body that was nulled (you called a setBodyXXXX function in the past), but you are setting it's individual inertial properties.");
    }
    rbdlModel_->mFixedBodies[bodyId]->SetIndividualInertialProperties(mass, centerOfMassInBodyFrame,inertiaAtCom, updateBodyInertia);
    return true;
  } else {
    unsigned int bodyId = bodyPtr->getBodyId();
    if (rbdlModel_->mBodies[bodyId]->GetChildrenWereAbsorbed()){
      MELO_WARN_THROTTLE_STREAM(1.0, "[RobotModel::setIndividualBodyInertiaProperties] The body "<<bodyPtr->getName()<<" absorbed its children (you called a setBodyXXXX function in the past), but you are setting it's individual inertia properties, instead of setBodyInertiaProperties.");
    }
    rbdlModel_->mBodies[bodyId]->SetIndividualInertialProperties(mass, centerOfMassInBodyFrame,inertiaAtCom, updateBodyInertia);
    return true;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::updateBodyInertia(BodyEnum body) const {
  const std::shared_ptr<romo::RigidBody<ConcreteDescription_>>& bodyPtr = this->bodyContainer_[body];
  if (bodyPtr->getIsFixedBody()){
    unsigned int bodyId = bodyPtr->getBodyId() - rbdlModel_->fixed_body_discriminator;
    rbdlModel_->mFixedBodies[bodyId]->UpdateInertialProperties();;
    return true;
  } else {
    unsigned int bodyId = bodyPtr->getBodyId();
    rbdlModel_->mBodies[bodyId]->UpdateInertialProperties();
    return true;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
const Eigen::Matrix3d& RobotModelRbdl<ConcreteDescription_, RobotState_>::getBodyInertiaMatrix(
    BodyEnum body) const
{
  return this->bodyContainer_[body]->getInertiaMatrix();
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::setBodyInertiaMatrix(BodyEnum body, const Eigen::Matrix3d& inertiaMatrix, bool updateBodyInertia) const {
  const std::shared_ptr<romo::RigidBody<ConcreteDescription_>>& bodyPtr = this->bodyContainer_[body];
  if (bodyPtr->getIsFixedBody()){
    MELO_ERROR_STREAM("[RobotModel::setBodyInertiaMatrix] You cannot use setBodyInertiaMatrix on a fixed body (called on body "<< bodyPtr->getName()<<"); use setIndividualBodyInertiaMatrix instead" );
    return false;
  } else {
    unsigned int bodyId = bodyPtr->getBodyId();
    rbdlModel_->mBodies[bodyId]->AbsorbFixedChildren(false);
    rbdlModel_->mBodies[bodyId]->SetIndividualInertia(inertiaMatrix, updateBodyInertia);
    return true;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::setIndividualBodyInertiaMatrix(BodyEnum body, const Eigen::Matrix3d& inertiaMatrix, bool updateBodyInertia) const {
  const std::shared_ptr<romo::RigidBody<ConcreteDescription_>>& bodyPtr = this->bodyContainer_[body];
  if (bodyPtr->getIsFixedBody()){
    unsigned int bodyId = bodyPtr->getBodyId() - rbdlModel_->fixed_body_discriminator;
    if (rbdlModel_->mFixedBodies[bodyId]->GetIsNulled()){
      MELO_WARN_THROTTLE_STREAM(1.0, "[RobotModel::setIndividualBodyInertiaMatrix] The body "<<bodyPtr->getName()<<" is a fixed body that was nulled (you called a setBodyXXXX function in the past), but you are setting it's individual inertia.");
    }
    rbdlModel_->mFixedBodies[bodyId]->SetIndividualInertia(inertiaMatrix, updateBodyInertia);
    return true;
  } else {
    unsigned int bodyId = bodyPtr->getBodyId();
    if (rbdlModel_->mBodies[bodyId]->GetChildrenWereAbsorbed()){
      MELO_WARN_THROTTLE_STREAM(1.0, "[RobotModel::setIndividualBodyInertiaMatrix] The body "<<bodyPtr->getName()<<" absorbed its children (you called a setBodyXXXX function in the past), but you are setting it's individual inertia, instead of setBodyInertiaMatrix.");
    }
    rbdlModel_->mBodies[bodyId]->SetIndividualInertia(inertiaMatrix, updateBodyInertia);
    return true;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getGravityTerms(Eigen::VectorXd& gravity) const {
  assert(gravity.size() == getDofCount());
  RigidBodyDynamics::GravityTerms(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
      gravity, false);
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::VectorXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getGravityTerms() const {
  Eigen::VectorXd gravity = Eigen::VectorXd::Zero(getDofCount());
  RigidBodyDynamics::GravityTerms(
      *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
      gravity, false);
  return gravity;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getNonlinearEffects(Eigen::VectorXd& nonlinearEffects) const {
  assert(nonlinearEffects.size() == getDofCount());
  RigidBodyDynamics::NonlinearEffects(*rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
                                      stateGeneralizedVelocitiesAngularRBDL_, nonlinearEffects, false);
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::VectorXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getNonlinearEffects() const {
  Eigen::VectorXd nonlinearEffects = Eigen::VectorXd::Zero(getDofCount());
  RigidBodyDynamics::NonlinearEffects(*rbdlModel_,
                                      stateGeneralizedPositionsQuaternionRBDL_,
                                      stateGeneralizedVelocitiesAngularRBDL_,
                                      nonlinearEffects, false);
  return nonlinearEffects;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getMassInertiaMatrix(Eigen::MatrixXd& massMatrix) const {
  assert(massMatrix.rows() == getDofCount() && massMatrix.cols() == getDofCount());
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(*rbdlModel_,
                                                 stateGeneralizedPositionsQuaternionRBDL_,
                                                 massMatrix,
                                                 false);

  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::MatrixXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getMassInertiaMatrix() const {
  Eigen::MatrixXd massMatrix = Eigen::MatrixXd::Zero(getDofCount(), getDofCount());
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(*rbdlModel_,
                                                 stateGeneralizedPositionsQuaternionRBDL_,
                                                 massMatrix, false);
  return massMatrix;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getGravityDynamicParameters(Eigen::VectorXd& parameters) const {
  // Copy the dynamic armDynamicParameters.
  const unsigned int numLinks = state_.getNumberOfJointVelocities();
  const unsigned int numParametersPerLink = 4;

  parameters = Eigen::VectorXd::Zero(numLinks*numParametersPerLink);
  unsigned int linkId = 0;
  for (auto body : this->bodyContainer_) {

    if (body->getBodyId() == rbdlModel_->rootId) {
      // Skip the root body.
      continue;
    }

    if (body->getIsFixedBody()) {
      // Skip if body is fixed.
      continue;
    }

    const double bodyMass = body->getMass();
    const Eigen::Vector3d positionBodyToBodyComInBodyFrame = body->getPositionBodyToBodyCom(CoordinateFrameEnum::BODY);

    parameters(numParametersPerLink*linkId + 0) = bodyMass;
    parameters.segment<3>(numParametersPerLink*linkId + 1) = positionBodyToBodyComInBodyFrame;

    linkId++;
  }
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getJointGravityTermsFromJointConfigurationAndLinkParameters(
    Eigen::VectorXd& gravityTerms, const Eigen::VectorXd& jointPositions, const Eigen::VectorXd& dynamicParams) {

  // Copy state since we are changing it.
  const auto jointPositionsCopy = state_.getJointPositions();

  // Copy the current armDynamicParameters.
  const unsigned int numLinks = state_.getNumberOfJointVelocities();
  const unsigned int numParametersPerLink = 4;
  Eigen::VectorXd dynamicParametersCopy = Eigen::VectorXd::Zero(numLinks*numParametersPerLink);
  unsigned int linkId = 0;
  for (auto body : this->bodyContainer_) {

    if (body->getBodyId() == rbdlModel_->rootId) {
      // Skip the root body.
      continue;
    }

    if (body->getIsFixedBody()) {
      // Skip if body is fixed.
      continue;
    }

    const double bodyMass = body->getMass();
    const Eigen::Vector3d positionBodyToBodyComInBodyFrame = body->getPositionBodyToBodyCom(CoordinateFrameEnum::BODY);

    dynamicParametersCopy(numParametersPerLink*linkId + 0) = bodyMass;
    dynamicParametersCopy.segment<3>(numParametersPerLink*linkId + 1) = positionBodyToBodyComInBodyFrame;

    linkId++;
  }

  // Update configuration.
  state_.setJointPositions(typename RobotState::JointPositions(jointPositions));
  this->setState(state_, true);

  // Update dynamic armDynamicParameters.
  linkId = 0;

  for (auto body : this->bodyContainer_) {
    if (body->getBodyId() == rbdlModel_->rootId) {
      // Skip the root body.
      continue;
    }

    if (body->getIsFixedBody()) {
      // Skip if body is fixed.
      continue;
    }

    setBodyMass(body->getBodyEnum(), dynamicParams(numParametersPerLink*linkId + 0), false);
    setPositionBodyToBodyCom(dynamicParams.segment<3>(numParametersPerLink*linkId + 1),body->getBodyEnum(), CoordinateFrameEnum::BODY, true);
    linkId++;
  }

  // Update gravity.
  gravityTerms = getGravityTerms().bottomRows(numLinks);

  // Reset the state.
  state_.setJointPositions(jointPositionsCopy);
  this->setState(state_, true);

  // Reset the dynamic armDynamicParameters.
  linkId = 0;
  for (auto body : this->bodyContainer_) {
    if (body->getBodyId() == rbdlModel_->rootId) {
      // Skip the root body.
      continue;
    }

    if (body->getIsFixedBody()) {
      // Skip if body is fixed.
      continue;
    }

    setBodyMass(body->getBodyEnum(), dynamicParametersCopy(numParametersPerLink*linkId + 0), false);
    setPositionBodyToBodyCom( dynamicParametersCopy.segment<3>(numParametersPerLink*linkId + 1),body->getBodyEnum(), CoordinateFrameEnum::BODY, true);
    linkId++;
  }

  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
const typename romo::RigidBody<ConcreteDescription_>& RobotModelRbdl<ConcreteDescription_, RobotState_>::getBody(
    BodyEnum body) const {
  return *this->bodyContainer_[body];
}

template<typename ConcreteDescription_, typename RobotState_>
const typename romo::RigidBody<ConcreteDescription_>& RobotModelRbdl<ConcreteDescription_, RobotState_>::getBody(
  BranchEnum branch,
  BodyNodeEnum node) const {
  return *this->bodyBranchNodeContainer_.at(branch).at(node);
}


} /* namespace romo */

