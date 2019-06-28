/*
 * ColliderManagerBullet.hpp
 *
 *  Created on: Apr 17, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <LinearMath/btVector3.h>

#include <bullet/btBulletCollisionCommon.h>

#include <BulletCollision/NarrowPhaseCollision/btGjkEpa2.h>

#include <collisions/ColliderManager.hpp>
#include <collisions/CollisionBody.hpp>

namespace collisions_bullet {

// Implements ColliderManager using Bullet
class ColliderManagerBullet: public collisions::ColliderManager {

public:

  // Constructs the ColliderManagerBullet
  explicit ColliderManagerBullet() = default;
  ~ColliderManagerBullet() override = default;


  collisions::DistanceResults checkDistance(const collisions::CollisionGroup& collision_group1,
                                            const collisions::CollisionGroup& collision_group2,
                                            const collisions::DistanceOptions& distance_options = collisions::DistanceOptions() ) override;

  collisions::CollisionResults checkCollision(const collisions::CollisionGroup& group1,
                                              const collisions::CollisionGroup& group2,
                                              const collisions::CollisionOptions& collision_options) override;

protected:

  using BulletDistanceResult = typename btGjkEpaSolver2::sResults;

  BulletDistanceResult getDistanceBetweenShapes(btCollisionShape* shape_1, btTransform transform_1,
                                                btCollisionShape* shape_2, btTransform transform_2);

};

}
