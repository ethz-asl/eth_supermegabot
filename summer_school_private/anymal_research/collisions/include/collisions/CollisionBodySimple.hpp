/*
 * CollisionBody.hpp
 *
 *  Created on: May 28, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <collisions/CollisionBody.hpp>

namespace collisions{

class CollisionBodySimple: public CollisionBody{
public:
  using Position = typename CollisionBody::Position;
  using Orientation = typename CollisionBody::Orientation;

public:
  CollisionBodySimple(const std::shared_ptr< collisions_geometry::CollisionGeometry >& col_geom,
                    const Position& position,
                    Orientation orientation,
                    unsigned int id):
                col_geom_ptr(col_geom),
                id_(id),
                position_(position),
                orientation_(orientation){}
  ~CollisionBodySimple() override = default;


  const std::shared_ptr<collisions_geometry::CollisionGeometry>& getCollisionGeometry() const override{
    return col_geom_ptr;
  }


  void setPositionInWorld(const Position& position){
    position_ = position;
  }

  Position getPositionInWorld() const override{
    return position_;
  }

  void setOrientationInWorld(const Orientation& orientation){
    orientation_ = orientation;
  }

  Orientation getOrientationInWorld() const override{
    return orientation_;
  }

  unsigned int getId() const override{
    return id_;
  }

private:

  const std::shared_ptr<collisions_geometry::CollisionGeometry> col_geom_ptr;

  const unsigned int id_;

  Position position_;
  Orientation orientation_;

};

}//collisions
