/*
 * conversions.hpp
 *
 *  Created on: Apr 17, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <LinearMath/btTransform.h>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>


#include <collisions/CollisionBody.hpp>

#include <collisions_geometry/CollisionGeometry.hpp>
#include <collisions_geometry/CollisionGeometryBox.hpp>
#include <collisions_geometry/CollisionGeometryCylinder.hpp>
#include <collisions_geometry/CollisionGeometrySphere.hpp>
#include <collisions_geometry/CollisionGeometryCombo.hpp> // Technically unused in the hpp, just here for completeness

#include <collisions_bullet/btCollisionObjectContainer.hpp>

namespace collisions_bullet {

class Converter {

public:

  Converter() = delete;
  ~Converter() = delete;


  static bool bulletObjectFromCollisionBody(
      const collisions::CollisionBody& collision_body,
      double margin,
      btCollisionObjectContainer& bullet_container_out);

  static bool bulletShapeFromCollisionGeometry(
      const collisions_geometry::CollisionGeometry& collision_geometry,
      double margin,
      btCollisionShape*& bullet_shape_out);



  static btBoxShape* createBtBoxShape(const collisions_geometry::CollisionGeometryBox&,
                                    double margin);

  static btCylinderShape* createBtCylinderShape(const collisions_geometry::CollisionGeometryCylinder&,
                                    double margin);

  static btSphereShape* createBtSphereShape(const collisions_geometry::CollisionGeometrySphere&,
                                      double margin);

  static btCompoundShape* createBtCompoundShape(const collisions_geometry::CollisionGeometryCombo&,
                                      double margin);

};


}
