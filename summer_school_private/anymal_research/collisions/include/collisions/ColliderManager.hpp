/*
 * ColliderManager.hpp
 *
 *  Created on: May 15, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <collisions_geometry/CollisionGeometry.hpp>
#include <collisions/CollisionGroup.hpp>
#include <collisions/CollisionResults.hpp>
#include <collisions/CollisionOptions.hpp>
#include <collisions/DistanceResults.hpp>
#include <collisions/DistanceOptions.hpp>


namespace collisions {

// The ColliderManager handles collision checking for a robot.
class ColliderManager {

public:
  // Constructs the Collider
  explicit ColliderManager() = default;
  virtual ~ColliderManager() = default;

  // Checks for collisions between two CollisionGroups.
  virtual collisions::CollisionResults
         checkCollision(const CollisionGroup& collision_group1,
                        const CollisionGroup& collision_group2,
                        const CollisionOptions& collision_options = CollisionOptions() ) = 0;


  virtual collisions::DistanceResults
         checkDistance(const CollisionGroup& collision_group1,
                       const CollisionGroup& collision_group2,
                       const DistanceOptions& distance_options = DistanceOptions()) = 0;

};

} // namespace collisions

