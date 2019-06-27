/*
 * CollisionBodyRomo.hpp
 *
 *  Created on: Apr 3, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <set>

#include <collisions_geometry/CollisionGeometry.hpp>

#include <collisions/CollisionBody.hpp>

#include <romo/common/RigidBody.hpp>

namespace romo{

template <typename ConcreteDescription_>
class CollisionBodyRomo: public collisions::CollisionBody{
public:
  using Position = typename collisions::CollisionBody::Position;
  using Orientation = typename collisions::CollisionBody::Orientation;
public:
  using RD = romo::RobotDescription<ConcreteDescription_>;
  using BodyEnum = typename RD::BodyEnum;
  using RigidBody = romo::RigidBody<ConcreteDescription_>;

public:
  CollisionBodyRomo(const std::shared_ptr< RigidBody >& rigid_body);
  ~CollisionBodyRomo() override = default;

  const std::shared_ptr<collisions_geometry::CollisionGeometry>& getCollisionGeometry() const override;

  Position getPositionInWorld() const override;
  Orientation getOrientationInWorld() const override;

  unsigned int getId() const override;

private:

  const std::shared_ptr< const RigidBody > rigid_body_ptr;

};

}//romo

#include <romo/common/CollisionBodyRomo.tpp>
