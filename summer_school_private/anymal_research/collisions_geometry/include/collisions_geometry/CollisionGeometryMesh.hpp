/*
 * CollisionGeometryMesh.hpp
 *
 *  Created on: May 18, 2017
 *      Author: Perry Franklin
 */

#include <collisions_geometry/CollisionGeometry.hpp>

#include <Eigen/Dense>

#pragma once

namespace collisions_geometry {

// Collision Geometry data for a Mesh. Uses a list of vertices and a list of face-indices 
//(eg face-indices (1,5,6) means use vertices (vertices_[1], vertices_[5], vertices_[6]).
class CollisionGeometryMesh: public CollisionGeometry {
 public:

 public:
  explicit CollisionGeometryMesh(): CollisionGeometry(Position(), Rotation()) {}
  ~CollisionGeometryMesh() override = default;

  GeomType getType() const override{
    return GeomType::MESH;
  }

  const std::vector< Eigen::Vector3d >& getVertices(){
    return vertices_;
  }

  const std::vector< Eigen::Vector3i >& getFaceIndices(){
    return face_indices_;
  }
  
  virtual CollisionGeometry* clone() const override{
    CollisionGeometryMesh* new_ptr = new CollisionGeometryMesh;
    new_ptr->vertices_ = vertices_;
    new_ptr->face_indices_ = face_indices_;

    return new_ptr;
  }


protected:

  std::vector< Eigen::Vector3d > vertices_;
  std::vector< Eigen::Vector3i > face_indices_;

};

}
