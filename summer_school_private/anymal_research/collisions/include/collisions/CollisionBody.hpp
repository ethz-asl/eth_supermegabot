/*
 * CollisionBody.hpp
 *
 *  Created on: Apr 3, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <memory>

#include <collisions_geometry/CollisionGeometry.hpp>

namespace collisions{

// Interface for collision objects to be collided.
class CollisionBody{
public:
  CollisionBody() = default;
  virtual ~CollisionBody() = default;

  using Position = typename collisions_geometry::CollisionGeometry::Position;
  using Orientation = typename collisions_geometry::CollisionGeometry::Rotation;

  virtual const std::shared_ptr<collisions_geometry::CollisionGeometry>& getCollisionGeometry() const = 0;

  virtual Position getPositionInWorld() const = 0;
  virtual Orientation getOrientationInWorld() const = 0;

  virtual unsigned int getId() const = 0;

};

using CollisionBodyPtr = std::shared_ptr<CollisionBody>;
using CollisionBodyConstPtr = std::shared_ptr<const CollisionBody>;

}//collisions


