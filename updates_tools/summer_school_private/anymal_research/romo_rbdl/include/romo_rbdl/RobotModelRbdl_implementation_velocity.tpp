/*
 * RobotModelRbdl_implementation_velocity.tpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Dario Bellicoso
 */

#include "romo_rbdl/rbdl_kinematics.hpp"
#include "romo_rbdl/RobotModelRbdl.hpp"

namespace romo_rbdl {

template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getLinearVelocityWorldToBody(BodyEnum body,
                                                                                                CoordinateFrameEnum frame) const {
  return getLinearVelocityWorldToPointOnBody(body, Eigen::Vector3d::Zero(), frame);
}

template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getLinearVelocityWorldToBody(BranchEnum branch,
                                                                                                BodyNodeEnum node,
                                                                                                CoordinateFrameEnum frame) const {
  return getLinearVelocityWorldToPointOnBody(branch, node, Eigen::Vector3d::Zero(), frame);
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getLinearVelocityWorldToPointOnBody(
  BodyEnum body,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  CoordinateFrameEnum frame) const {
  return getLinearVelocityWorldToPointOnBody(this->bodyContainer_[body]->getBranchEnum(),
                                             this->bodyContainer_[body]->getBodyNodeEnum(),
                                             positionBodyToPointOnBodyInBodyFrame, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getLinearVelocityWorldToPointOnBody(
  BranchEnum branch,
  BodyNodeEnum node,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  CoordinateFrameEnum frame) const {

  Eigen::Vector3d velocity = RigidBodyDynamics::CalcPointVelocity(*rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
                                                                  stateGeneralizedVelocitiesAngularRBDL_,
                                                                  this->bodyBranchNodeContainer_.at(branch).at(node)->getBodyId(),
                                                                  positionBodyToPointOnBodyInBodyFrame, false);
  switch(frame) {
    case CoordinateFrameEnum::WORLD:
      // Do nothing
      break;
    case CoordinateFrameEnum::BASE:
      velocity = this->bodyContainer_[BodyEnum::BASE]->getOrientationWorldToBody()*velocity;
      break;
    default:
      std::cout<<"[RobotModelRBDL:getVelocityWorldToPointOnBody] Frame not supported."<<std::endl;
      break;
  }

  return velocity;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getLinearVelocityBodyToPointOnBody(
  BodyEnum fromBody,
  BodyEnum toBody,
  const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
  CoordinateFrameEnum frame) const {
  return (getLinearVelocityWorldToPointOnBody(toBody, positionBodyToPointOnBodyInBodyFrame, frame) -
    getLinearVelocityWorldToPointOnBody(fromBody, Eigen::Vector3d::Zero(), frame));
}

template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getLinearVelocityWorldToBodyCom(
    BodyEnum body, CoordinateFrameEnum frame) const {
  return getLinearVelocityWorldToPointOnBody(
      body,
      getBody(body).getPositionBodyToBodyCom(CoordinateFrameEnum::BODY),
      frame);
}

template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getLinearVelocityWorldToCom(CoordinateFrameEnum frame) const {
  Eigen::Vector3d linearVelocityComInWorldFrame = Eigen::Vector3d::Zero();
  for (const auto& body : this->bodyContainer_) {
    if (!rbdlModel_->IsFixedBodyId(body->getBodyId())) {
      linearVelocityComInWorldFrame += body->getMass()
                                     * RigidBodyDynamics::CalcPointVelocity(*rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
                                                                            stateGeneralizedVelocitiesAngularRBDL_, body->getBodyId(),
                                                                            body->getPositionBodyToBodyCom(CoordinateFrameEnum::BODY), false);
    }
  }

  linearVelocityComInWorldFrame /= this->getTotalMass();

  switch (frame) {
   case (CoordinateFrameEnum::WORLD):
       return linearVelocityComInWorldFrame;
   case(CoordinateFrameEnum::BASE): {
     const Eigen::Matrix3d& orientationWorldToBase = getOrientationWorldToBody(BodyEnum::BASE);
     return orientationWorldToBase*(linearVelocityComInWorldFrame);
   }
   default: throw std::runtime_error("[RobotModel::getLinearVelocityCom] frame is not supported!");
  }

}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getAngularVelocityWorldToBody(
  BodyEnum body,
  CoordinateFrameEnum frame) const {
  return getAngularVelocityWorldToBody(this->bodyContainer_[body]->getBranchEnum(),
                                       this->bodyContainer_[body]->getBodyNodeEnum(),
                                       frame);
}

template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getAngularVelocityWorldToBody(
    BranchEnum branch,
    BodyNodeEnum bodyNode,
    CoordinateFrameEnum frame) const {
  Eigen::Vector3d angularVelocityInWorldFrame =
      RigidBodyDynamics::CalcBodyAngularVelocityInWorldFrame(
          *rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_,
          stateGeneralizedVelocitiesAngularRBDL_,
          this->bodyBranchNodeContainer_.at(branch).at(bodyNode)->getBodyId(),
          false);
  switch (frame) {
    case (CoordinateFrameEnum::WORLD): {
      return angularVelocityInWorldFrame;
    }
    case(CoordinateFrameEnum::BASE): {
      const Eigen::Matrix3d& orientationWorldToBase = getOrientationWorldToBody(BodyEnum::BASE);
      return orientationWorldToBase*(angularVelocityInWorldFrame);
    }
    default: throw std::runtime_error("[RobotModel::getAngularVelocityBody] frame is not supported!");
  }
}


} /* namespace romo */
