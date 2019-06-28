/*
 * CollisionGeometryCombo.hpp
 *
 *  Created on: Aug 15, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <collisions_geometry/CollisionGeometry.hpp>

#include <memory>

namespace collisions_geometry {

// Collision Geometry data which is a combination of other geometry objects.
class CollisionGeometryCombo : public CollisionGeometry {

public:
  explicit CollisionGeometryCombo():
    CollisionGeometry(CollisionGeometry::Position() ,CollisionGeometry::Rotation())
    {}
  ~CollisionGeometryCombo() override = default;

  GeomType getType() const override {
    return GeomType::COMBO;
  }

  const std::vector< std::unique_ptr<CollisionGeometry> >& getGeometries() const{
    return geometries;
  }

  // Takes ownership of a geometry. The provided unique_ptr will become null.
  void addGeometry( std::unique_ptr<CollisionGeometry>& geometry ){
    geometries.push_back( std::unique_ptr<CollisionGeometry>() );
    (--geometries.end())->swap(geometry);
  }


  CollisionGeometry* clone() const override{
    CollisionGeometryCombo* new_ptr =  new CollisionGeometryCombo;

    for (const std::unique_ptr<CollisionGeometry>& geometry: geometries){
      std::unique_ptr<CollisionGeometry> new_child_ptr(geometry->clone());
      new_ptr->addGeometry(new_child_ptr);
    }

    return new_ptr;
  }


protected:

  std::vector< std::unique_ptr<CollisionGeometry> > geometries;

};


}  // namespace collisions_geometry
