/*
 * CollisionGeometryCapsule.hpp
 *
 *  Created on: July 4, 2017
 *      Author: Perry Franklin
 */

#include <collisions_geometry/CollisionGeometry.hpp>

#pragma once

namespace collisions_geometry {

// Collision Geometry data for a Capsule. Adds a Radius and Length on z-axis of a cylinder, centered at the CollisionGeometry's frame.
class CollisionGeometryCapsule: public CollisionGeometry {
 public:

 public:
  explicit CollisionGeometryCapsule(const double& radius, const double& length,
                                     const Position& position,
                                     const Rotation& orientation):
                                    CollisionGeometry(position, orientation) {
                                      data[0] = radius;
                                      data[1] = length;
                                    }
  ~CollisionGeometryCapsule() override = default;

  GeomType getType() const override{
    return GeomType::CAPSULE;
  }

  double getRadius() const{
    return data[0];
  }

  double getLength() const{
    return data[1];
  }

  CollisionGeometry* clone() const override{
    return new CollisionGeometryCapsule(getRadius(), getLength(),
                                        getPosition(), getRotation());
  }

protected:

  // Data order is data[0]: radius
  //               data[1]: length along the Z axis
  std::array<double, 2> data;

};

}
