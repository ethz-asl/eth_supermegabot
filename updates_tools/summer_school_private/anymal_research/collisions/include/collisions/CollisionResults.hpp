/*
 * ColliderResults.hpp
 *
 *  Created on: May 15, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <vector>
#include <utility>

#include <collisions/CollisionBody.hpp>

namespace collisions {

// The CollisionResults class records results from a collision check. It (can) track:
// Whether a collision has occured;
// Which bodies are colliding.
// The locations of collision;
// Note that it is templated on the Collision Groups that are being collided; this is so
// that the results can return BodyEnums for whatever type of RigidBodies are collided.
class CollisionResults {
 public:
  typedef std::pair<unsigned int, unsigned int> CollidingPair;

 public:
  explicit CollisionResults():
  calculate_all_collisions(false),
  contacts_calculated(false)
  {};
  virtual ~CollisionResults() = default;

  bool calculate_all_collisions;
  std::vector< CollidingPair > colliding_pairs;


  bool contacts_calculated;
  std::vector< Eigen::Vector3d > contact_locations;

  // Checks if there are any colliding objects.
  bool colliding() const{
    return !colliding_pairs.empty();
  }

};

}
