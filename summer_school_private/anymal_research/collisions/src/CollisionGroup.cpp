/*
 * CollisionGroupBodies.cpp
 *
 *  Created on: Jul 18, 2017
 *      Author: Perry Franklin
 */

#include <collisions/CollisionGroup.hpp>

namespace collisions {

const std::vector< CollisionBodyConstPtr >& CollisionGroup::getBodies() const {
  return bodies_;
}

void CollisionGroup::addCollisionBody(const CollisionBodyPtr& collision_body_ptr ){
  bodies_.emplace_back(collision_body_ptr);
}

void CollisionGroup::addCollisionConstBody(const CollisionBodyConstPtr& collision_body_ptr ){
  bodies_.emplace_back(collision_body_ptr);
}

std::size_t CollisionGroup::size() const{
  return bodies_.size();
}

void CollisionGroup::clear() {
  bodies_.clear();
}

} // namespace collisions
