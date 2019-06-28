/*
 * RigidBodyRbdl.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Dario Bellicoso
 */

#include "romo_rbdl/RigidBodyRbdl.hpp"

namespace romo_rbdl {

template<typename ConcreteDescription_>
RigidBodyRbdl<ConcreteDescription_>::RigidBodyRbdl( std::shared_ptr<RigidBodyDynamics::Model> model, BodyEnum body)
    : Base(),
      model_(model),
      bodyEnum_(body),
      bodyId_(0),
      isFixedBody_(false)

{
  // Assign body id according to the RBDL model.
  auto it = model_->mBodyNameMap.find(RD::mapKeyEnumToKeyName(bodyEnum_));
  if (it == model_->mBodyNameMap.end()) {
    throw std::range_error("[RigidBodyRbdl::init] Body name '" + std::string(RD::mapKeyEnumToKeyName(bodyEnum_)) + "' was not found in map!");
  } else {
    bodyId_ = it->second;
  }

  // Check if the body is a fixed one.
  isFixedBody_ = model_->IsFixedBodyId(bodyId_);
}

template<typename ConcreteDescription_>
double RigidBodyRbdl<ConcreteDescription_>::getMass() const {
  if (isFixedBody_) {
//    throw std::runtime_error("Asking for fixed body (" + this->getName() + ") mass.");
    return 0.0;
  } else {
    return model_->mBodies[bodyId_]->GetMass();
  }
}
template<typename ConcreteDescription_>
Eigen::Vector3d RigidBodyRbdl<ConcreteDescription_>::getPositionWorldToBodyCom(
    const CoordinateFrameEnum& frame) const
{
  if (isFixedBody_) {
    throw std::runtime_error(
        "[RigidBodyRbdl::getPositionWorldToBodyCom] Asking for fixed body (" + std::string(RD::mapKeyEnumToKeyName(bodyEnum_)) + ") center of mass.");
  }

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      return RigidBodyDynamics::CalcBodyToBaseCoordinates(
          *model_, RigidBodyDynamics::Math::VectorNd(), bodyId_,
          model_->mBodies[bodyId_]->GetCenterOfMass(), false);
    }
    case (CoordinateFrameEnum::BASE): {
      return (model_->X_base[model_->rootId].E
          * RigidBodyDynamics::CalcBodyToBaseCoordinates(
              *model_, RigidBodyDynamics::Math::VectorNd(), bodyId_,
              model_->mBodies[bodyId_]->GetCenterOfMass(), false));
    }
    default:
      throw std::runtime_error(
          "[RigidBodyRbdl::getPositionWorldToBodyCom] Invalid frame.");
  }
}

template<typename ConcreteDescription_>
Eigen::Vector3d RigidBodyRbdl<ConcreteDescription_>::getPositionBodyToBodyCom(
      const CoordinateFrameEnum& frame) const {
  if (isFixedBody_) {
    throw std::runtime_error(
        "[RigidBodyRbdl::getPositionBodyToBodyCom] Asking for fixed body (" + std::string(RD::mapKeyEnumToKeyName(bodyEnum_)) + ") center of mass.");
  }

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      return (getOrientationWorldToBody().transpose()*model_->mBodies[bodyId_]->GetCenterOfMass());
    }

    case (CoordinateFrameEnum::BASE): {
      return (model_->X_base[model_->rootId].E*(getOrientationWorldToBody().transpose()*model_->mBodies[bodyId_]->GetCenterOfMass()));
    }

    case (CoordinateFrameEnum::BODY): {
      return model_->mBodies[bodyId_]->GetCenterOfMass();
    }

    default:
      throw std::runtime_error("[RigidBodyRbdl::getPositionBodyToBodyCom] Invalid frame.");
  }
}

template<typename ConcreteDescription_>
const Eigen::Matrix3d& RigidBodyRbdl<ConcreteDescription_>::getInertiaMatrix() const {
  if (isFixedBody_){
    throw std::runtime_error("[RigidBodyRbdl::getInertiaMatrix] Asking for fixed body (" + std::string(RD::mapKeyEnumToKeyName(bodyEnum_)) + ") inertia matrix.");
  }
  return model_->mBodies[bodyId_]->GetInertia();
}

template<typename ConcreteDescription_>
Eigen::Vector3d RigidBodyRbdl<ConcreteDescription_>::getPositionWorldToBody(
    const CoordinateFrameEnum& frame) const
{
  return getPositionWorldToPointOnBody(Eigen::Vector3d::Zero(), frame);
}

template<typename ConcreteDescription_>
Eigen::Vector3d RigidBodyRbdl<ConcreteDescription_>::getPositionWorldToPointOnBody(const Eigen::Vector3d& pointOnBody,
    const CoordinateFrameEnum& frame) const
{
  const Eigen::Vector3d positionWorldToPointOnBodyInWorldFrame =
      RigidBodyDynamics::CalcBodyToBaseCoordinates(
          *model_, RigidBodyDynamics::Math::VectorNd(), bodyId_, pointOnBody, false);

  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      return positionWorldToPointOnBodyInWorldFrame;
    }
    case (CoordinateFrameEnum::BASE): {
      return (model_->X_base[model_->rootId].E * positionWorldToPointOnBodyInWorldFrame);
    }
    case (CoordinateFrameEnum::BODY): {
      return (getOrientationWorldToBody() * positionWorldToPointOnBodyInWorldFrame);
    }
    default:
      throw std::runtime_error(
          "[RigidBodyRbdl::getPositionBodyToBodyCom] Invalid frame.");
  }

  MELO_ERROR("[RigidBody::getPositionWorldToPointOnBody] Error while computing the position vector.");
  return Eigen::Vector3d::Zero();
}


template<typename ConcreteDescription_>
const Eigen::Matrix3d& RigidBodyRbdl<ConcreteDescription_>::getOrientationWorldToBody() const {
  if (this->isFixedBody_) {
    const unsigned int fbody_id = bodyId_ - model_->fixed_body_discriminator;
    model_->mFixedBodies[fbody_id]->mBaseTransform =
        model_->mFixedBodies[fbody_id]->mParentTransform
      * model_->X_base[model_->mFixedBodies[fbody_id]->mMovableParent];

    return model_->mFixedBodies[fbody_id]->mBaseTransform.E;
  }

  return model_->X_base[bodyId_].E;
}


} /* namespace romo_rbdl */
