/*
 * CollisionBodyRomo.tpp
 *
 *  Created on: Apr 3, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <set>

#include <collisions_geometry/CollisionGeometry.hpp>

#include <collisions/CollisionBody.hpp>

#include <romo/common/RobotDescription.hpp>

namespace romo{

template <typename ConcreteDescription_>
CollisionBodyRomo<ConcreteDescription_>::CollisionBodyRomo(const std::shared_ptr< RigidBody >& rigid_body):
  collisions::CollisionBody(),
  rigid_body_ptr(rigid_body){}

template <typename ConcreteDescription_>
const std::shared_ptr<collisions_geometry::CollisionGeometry>& CollisionBodyRomo<ConcreteDescription_>::getCollisionGeometry() const{
  return rigid_body_ptr->collision_geometry;
}

template <typename ConcreteDescription_>
typename CollisionBodyRomo<ConcreteDescription_>::Position CollisionBodyRomo<ConcreteDescription_>::getPositionInWorld() const{
  return Position(rigid_body_ptr->getPositionWorldToBody());
}

template <typename ConcreteDescription_>
typename CollisionBodyRomo<ConcreteDescription_>::Orientation CollisionBodyRomo<ConcreteDescription_>::getOrientationInWorld() const{
  return Orientation(rigid_body_ptr->getOrientationWorldToBody().transpose());
}

template <typename ConcreteDescription_>
unsigned int CollisionBodyRomo<ConcreteDescription_>::getId() const{
  return static_cast<unsigned int>(rigid_body_ptr->getBodyEnum());
}


}//romo


