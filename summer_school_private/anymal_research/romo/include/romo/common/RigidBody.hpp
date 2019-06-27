/*
 * RigidBody.hpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>

// romo
#include "romo/common/RobotDescription.hpp"

#include <collisions_geometry/CollisionGeometry.hpp>

// shared_ptr
#include <memory>

namespace romo {

template<typename ConcreteDescription_>
class RigidBody {

 protected:
  using RD = RobotDescription<ConcreteDescription_>;

 public:
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;
  using BodyEnum = typename RD::BodyEnum ;
  using BranchEnum = typename RD::BranchEnum ;
  using BodyNodeEnum = typename RD::BodyNodeEnum;

  RigidBody() = default;
  virtual ~RigidBody() = default;

  // Get the world to body position in world or main body frame
  virtual Eigen::Vector3d getPositionWorldToBody(const CoordinateFrameEnum& frame = CoordinateFrameEnum::WORLD) const = 0;

  // Get the world to point on body position in world or main body frame
  virtual Eigen::Vector3d getPositionWorldToPointOnBody(const Eigen::Vector3d& pointOnBody, const CoordinateFrameEnum& frame = CoordinateFrameEnum::WORLD) const = 0;

  // Get the world to body center of mass position in world or main body frame
  virtual Eigen::Vector3d getPositionWorldToBodyCom(const CoordinateFrameEnum& frame = CoordinateFrameEnum::WORLD) const = 0;

  // Get the world to body center of mass position in world or main body frame
  virtual Eigen::Vector3d getPositionBodyToBodyCom(const CoordinateFrameEnum& frame = CoordinateFrameEnum::WORLD) const = 0;

  // Get the body inertia matrix
  virtual const Eigen::Matrix3d& getInertiaMatrix() const = 0;

  // Get the body mass
  virtual double getMass() const = 0;

  // Get the rotation matrix which rotates a vector from world frame to body frame
  virtual const Eigen::Matrix3d& getOrientationWorldToBody() const = 0;

  // Get body enum corresponding to body
  virtual BodyEnum getBodyEnum() const = 0;

  // Get branch enum corresponding to body
  virtual BranchEnum getBranchEnum() const = 0;

  // Get body node enum corresponding to body
  virtual BodyNodeEnum getBodyNodeEnum() const = 0;

  // Get name of the body
  virtual std::string getName() const = 0;

  // Get a unique id for this RigidBody
  virtual unsigned int getBodyId() const = 0;

  // Get isFixed property
  virtual bool getIsFixedBody() const = 0;

  // pointer to the collision geometry 
  std::shared_ptr<collisions_geometry::CollisionGeometry> collision_geometry;
};

} /* namespace romo */


