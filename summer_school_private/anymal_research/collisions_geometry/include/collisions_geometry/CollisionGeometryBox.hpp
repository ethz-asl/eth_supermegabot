/*
 * CollisionGeometryBox.hpp
 *
 *  Created on: May 18, 2017
 *      Author: Perry Franklin
 */

#include <collisions_geometry/CollisionGeometry.hpp>

#pragma once

namespace collisions_geometry {

// Collision Geometry data for a Box, centered at the CollisionGeometry's frame.
class CollisionGeometryBox: public CollisionGeometry {
 public:

 public:
  explicit CollisionGeometryBox(double x_size, double y_size, double z_size,
    const Position& position, const Rotation& orientation): CollisionGeometry(position, orientation) {
    data[0] = x_size;
    data[1] = y_size;
    data[2] = z_size;
  };
  ~CollisionGeometryBox() override = default;

  GeomType getType() const override {
    return GeomType::BOX;
  }

  double getXSize() const{
    return data[0];
  }

  double getYSize() const{
    return data[1];
  }

  double getZSize() const{
    return data[2];
  }

  CollisionGeometry* clone() const override{
    return new CollisionGeometryBox(getXSize(), getYSize(), getZSize(),
                                    getPosition(), getRotation());
  }

protected:

  // Data order is data[0]: x size
  //               data[1]: y size
  //               data[2]: z size
  std::array<double, 3> data;

};

}
