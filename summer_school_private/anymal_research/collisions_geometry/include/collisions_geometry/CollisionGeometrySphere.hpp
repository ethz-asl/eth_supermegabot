/*
 * CollisionGeometrySphere.hpp
 *
 *  Created on: Apr 5, 2018
 *      Author: Perry Franklin
 */

#include <collisions_geometry/CollisionGeometry.hpp>

#pragma once

namespace collisions_geometry {

// Collision Geometry data for a Sphere centered at the CollisionGeometry's frame.
class CollisionGeometrySphere: public CollisionGeometry {
 public:

 public:
  explicit CollisionGeometrySphere(const double& radius,
                                   const Position& position,
                                   const Rotation& orientation):
                                 CollisionGeometry(position, orientation) {
                                      data[0] = radius;
                                    }
  ~CollisionGeometrySphere() override = default;

  GeomType getType() const override{
    return GeomType::SPHERE;
  }

  double getRadius() const{
    return data[0];
  }

  CollisionGeometry* clone() const override{
    return new CollisionGeometrySphere(getRadius(),
                                       getPosition(), getRotation());
  }

protected:

  // Data order is data[0]: radius
  std::array<double, 1> data;

};

}
