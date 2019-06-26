/*
 * conversions.cpp
 *
 *  Created on: Apr 17, 2018
 *      Author: Perry Franklin
 */

#include <collisions_bullet/conversions.hpp>

namespace collisions_bullet {


bool Converter::bulletObjectFromCollisionBody(
    const collisions::CollisionBody& collision_body,
    double margin,
    btCollisionObjectContainer& bullet_container_out){

  btCollisionShape* bullet_shape = nullptr;
  if (!collision_body.getCollisionGeometry()){
    return false;
  }
  if (!bulletShapeFromCollisionGeometry(*collision_body.getCollisionGeometry(), margin, bullet_shape)){
    return false;
  }

  collisions_geometry::CollisionGeometry::Transform rigid_body_transform(collision_body.getPositionInWorld(),collision_body.getOrientationInWorld());

  const collisions_geometry::CollisionGeometry& geom = *collision_body.getCollisionGeometry();

  collisions_geometry::CollisionGeometry::Transform col_geometry_transform(
      collisions_geometry::CollisionGeometry::Position(geom.getTransform().getPosition()),
      collisions_geometry::CollisionGeometry::Rotation(geom.getTransform().getRotation()));

  collisions_geometry::CollisionGeometry::Transform world_transform =
      rigid_body_transform * col_geometry_transform;

  std::shared_ptr<btCollisionObject> bullet_object = std::make_shared<btCollisionObject>();
  bullet_object->setCollisionShape(bullet_shape);

  kindr::RotationQuaternionD kindr_quat(world_transform.getRotation());

  btQuaternion bullet_quat(kindr_quat.x(), kindr_quat.y(), kindr_quat.z(), kindr_quat.w());
  btVector3 bullet_vec(world_transform.getPosition().x(),world_transform.getPosition().y(), world_transform.getPosition().z());

  btTransform bullet_transform(bullet_quat, bullet_vec);

  bullet_object->setWorldTransform(bullet_transform);

  // Now that it's safely in the container, we don't have to worry about deleting anything. Hopefully.
  bullet_container_out.setCollisionObject(bullet_object);

  return true;
}

bool Converter::bulletShapeFromCollisionGeometry(
    const collisions_geometry::CollisionGeometry& collision_geometry,
    double margin,
    btCollisionShape*& bullet_shape_out){

  switch(collision_geometry.getType()){

  case collisions_geometry::CollisionGeometry::GeomType::BOX:
    {
      const collisions_geometry::CollisionGeometryBox& box = dynamic_cast<const collisions_geometry::CollisionGeometryBox&>(collision_geometry);
      bullet_shape_out = createBtBoxShape(box, margin);
    }
    break;

  case collisions_geometry::CollisionGeometry::GeomType::CYLINDER:
    {
      const collisions_geometry::CollisionGeometryCylinder& cyl = dynamic_cast<const collisions_geometry::CollisionGeometryCylinder&>(collision_geometry);
      bullet_shape_out = createBtCylinderShape(cyl, margin);
    }
    break;

  case collisions_geometry::CollisionGeometry::GeomType::SPHERE:
    {
      const collisions_geometry::CollisionGeometrySphere& cyl = dynamic_cast<const collisions_geometry::CollisionGeometrySphere&>(collision_geometry);
      bullet_shape_out = createBtSphereShape(cyl, margin);
    }
    break;

  case collisions_geometry::CollisionGeometry::GeomType::COMBO:
    {
      const collisions_geometry::CollisionGeometryCombo& combo = dynamic_cast<const collisions_geometry::CollisionGeometryCombo&>(collision_geometry);
      bullet_shape_out = createBtCompoundShape(combo, margin);
    }
    break;

  default:

    std::cerr<<"Bullet Converter::bulletShapeFromCollisionGeometry(..) got an unsupported geometry type"<<std::endl;
    return false;
  }

  bullet_shape_out->setMargin(0.0);

  return true;
}


btBoxShape* Converter::createBtBoxShape(const collisions_geometry::CollisionGeometryBox& box,
                                  double margin){
  return new btBoxShape(btVector3(box.getXSize()/2.0+margin,
                                  box.getYSize()/2.0+margin,
                                  box.getZSize()/2.0+margin));
}

btCylinderShape* Converter::createBtCylinderShape(const collisions_geometry::CollisionGeometryCylinder& cyl,
                                  double margin){
  return new btCylinderShapeZ(btVector3(cyl.getRadius()+margin, cyl.getRadius()+margin, cyl.getLength()/2.0+margin));
}

btSphereShape* Converter::createBtSphereShape(const collisions_geometry::CollisionGeometrySphere& sphere,
                                     double margin){
  return new btSphereShape(sphere.getRadius()+margin);
}

btCompoundShape* Converter::createBtCompoundShape(const collisions_geometry::CollisionGeometryCombo& combo,
                                                         double margin){

  btCompoundShape* bullet_compound_shape = new btCompoundShape;

  for(const std::unique_ptr<collisions_geometry::CollisionGeometry>& geom : combo.getGeometries()){

    btCollisionShape* bullet_collision_shape;

    bulletShapeFromCollisionGeometry(*geom, margin, bullet_collision_shape);

    kindr::RotationQuaternionD kindr_quat(geom->getRotation());

    btQuaternion bullet_quat(kindr_quat.x(), kindr_quat.y(), kindr_quat.z(), kindr_quat.w());
    btVector3 bullet_vec(geom->getPosition().x(),geom->getPosition().y(), geom->getPosition().z());

    btTransform bullet_transform(bullet_quat, bullet_vec);

    bullet_compound_shape->addChildShape(bullet_transform, bullet_collision_shape);

  }

  return bullet_compound_shape;
}
  

} // namespace collisions_bullet
