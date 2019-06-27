/*
 * CollisionGroup.hpp
 *
 *  Created on: Jul 18, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <set>

#include <collisions/CollisionBody.hpp>

namespace collisions {

// The CollisionGroup class designates some list of RigidBodies that need to be collided together.
// It contains a RigidBodyShPtrContainer which points at whatever RigidBodies need to be collided. The
// bodies to be collided are identified by their body Enums.

class CollisionGroup {

 public:
  explicit CollisionGroup() = default;
  virtual ~CollisionGroup() = default;

  const std::vector< CollisionBodyConstPtr >& getBodies() const;

  void addCollisionBody(const CollisionBodyPtr& collision_body_ptr );
  void addCollisionConstBody(const CollisionBodyConstPtr& collision_body_ptr );

  void clear();

  // Gets the number of bodies that are in this group.
  std::size_t size() const;

 private:

  std::vector< CollisionBodyConstPtr > bodies_;


};

}

