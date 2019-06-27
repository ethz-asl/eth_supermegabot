/*
 * RigidBodyRbdl.hpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// any_rbdl
#include <any_rbdl/rbdl.h>

// eigen
#include <Eigen/Core>

// romo
#include "romo/common/RigidBody.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// shared_ptr
#include <memory>

namespace romo_rbdl {

template<typename ConcreteDescription_>
class RigidBodyRbdl : public romo::RigidBody<ConcreteDescription_> {

  using Base = romo::RigidBody<ConcreteDescription_>;

 public:
  using RD = typename Base::RD;
  using BodyEnum        = typename RD::BodyEnum;
  using BodyNodeEnum    = typename RD::BodyNodeEnum;
  using BranchEnum      = typename RD::BranchEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;

  RigidBodyRbdl() = default;
  RigidBodyRbdl(std::shared_ptr<RigidBodyDynamics::Model> model, BodyEnum body);
  virtual ~RigidBodyRbdl() = default;

  // Get the world to body position in world or main body frame
  Eigen::Vector3d getPositionWorldToBody(const CoordinateFrameEnum& frame = CoordinateFrameEnum::WORLD) const override;

  // Get the world to body position in world or main body frame
  Eigen::Vector3d getPositionWorldToPointOnBody(const Eigen::Vector3d& pointOnBody, const CoordinateFrameEnum& frame = CoordinateFrameEnum::WORLD) const override;

  // Get the world to body center of mass position in world or main body frame
  Eigen::Vector3d getPositionWorldToBodyCom(const CoordinateFrameEnum& frame = CoordinateFrameEnum::WORLD) const override;

  // Get the world to body center of mass position in world or main body frame
  Eigen::Vector3d getPositionBodyToBodyCom(const CoordinateFrameEnum& frame = CoordinateFrameEnum::WORLD) const override;

  // Get the body mass
  double getMass() const override;

  // Get the body inertia matrix
  const Eigen::Matrix3d& getInertiaMatrix() const override;

  // Get the rotation matrix which rotates a vector from world frame to body frame
  const Eigen::Matrix3d& getOrientationWorldToBody() const override;

  // Get the body id assigned by RBDL
  unsigned int getBodyId() const override { return bodyId_; }

  // Get isFixed property
  bool getIsFixedBody() const override { return isFixedBody_; }

  // Get bodyEnum property
  BodyEnum getBodyEnum() const override { return bodyEnum_; }

  // Get branch enum
  BranchEnum getBranchEnum() const override { return RD::template mapEnums<BranchEnum>(bodyEnum_); }

  // Get body node enum
  BodyNodeEnum getBodyNodeEnum() const override { return RD::template mapEnums<BodyNodeEnum>(bodyEnum_); }

  // Get name of the body
  std::string getName() const override { return RD::template mapKeyEnumToKeyName(bodyEnum_); }

 protected:
  //! A pointer to the robot model core library.
  std::shared_ptr<RigidBodyDynamics::Model> model_;

  //! The body enum of the body
  BodyEnum bodyEnum_;

  //! The id of this body.
  unsigned int bodyId_;

  //! If true, this body is fixed on its parent body.
  bool isFixedBody_;

};

} /* namespace romo */

#include <romo_rbdl/RigidBodyRbdl.tpp>
