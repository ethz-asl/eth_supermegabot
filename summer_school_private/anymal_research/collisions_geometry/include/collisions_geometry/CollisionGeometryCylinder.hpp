/*
 * CollisionGeometryCylinder.hpp
 *
 *  Created on: May 18, 2017
 *      Author: Perry Franklin
 */

#include <collisions_geometry/CollisionGeometry.hpp>

#pragma once

namespace collisions_geometry {

// Collision Geometry data for a Cylinder. Adds a Radius and Length on z-axis of a cylinder, centered at the CollisionGeometry's frame.
class CollisionGeometryCylinder: public CollisionGeometry {
 public:

 public:
  explicit CollisionGeometryCylinder(const double& radius, const double& length,
                                     const Position& position,
                                     const Rotation& orientation):
                                    CollisionGeometry(position, orientation) {
                                      data[0] = radius;
                                      data[1] = length;
                                    }
  ~CollisionGeometryCylinder() override = default;

  GeomType getType() const override{
    return GeomType::CYLINDER;
  }

  double getRadius() const{
    return data[0];
  }

  double getLength() const{
    return data[1];
  }


  CollisionGeometry* clone() const override{
    return new CollisionGeometryCylinder(getRadius(), getLength(),
                                         getPosition(), getRotation());
  }

protected:

  // Data order is data[0]: raduis
  //               data[1]: length along the Z axis
  std::array<double, 2> data;

};

}
