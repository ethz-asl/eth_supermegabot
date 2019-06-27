/*
 * urdf_to_collisions_geometry.cpp
 *
 *  Created on: Aug 24, 2017
 *      Author: Perry Franklin
 */

#include <iostream>
#include <fstream>

#include <collisions_urdf/urdf_to_collisions_geometry.hpp>

#include <collisions_geometry/CollisionGeometryBox.hpp>
#include <collisions_geometry/CollisionGeometryCylinder.hpp>
#include <collisions_geometry/CollisionGeometrySphere.hpp>
#include <collisions_geometry/CollisionGeometryCombo.hpp>

#include <memory>

#include <urdf_parser/urdf_parser.h>

using namespace collisions_geometry;

namespace collisions_urdf{

// What I really want here is a template template parameter, ala
//   template < template <typenmae> class smart_ptr>
// but unfortunately unqiue_ptr and shared_ptr don't have the same template structure.
// IE unique_ptr needs
//   template < template <typename, typename> class smart_ptr >
// and shared_ptr takes
//   template <template <typename> class smart_ptr>
template <class smart_ptr_withCollisionGeometry >
bool setGeomFromUrdfCollision(const urdf::CollisionSharedPtr& urdf_collision, smart_ptr_withCollisionGeometry & geom_ptr,
                              const Position& link_position_offset,
                              const Orientation& link_orientation_offset){

  bool success = false;

  Position position( urdf_collision->origin.position.x,urdf_collision->origin.position.y,urdf_collision->origin.position.z );
  kindr::RotationQuaternionD quaternion(urdf_collision->origin.rotation.w, urdf_collision->origin.rotation.x, urdf_collision->origin.rotation.y, urdf_collision->origin.rotation.z);
  Orientation rotation(quaternion);

  position = position + rotation.rotate(link_position_offset);
  rotation = rotation*link_orientation_offset;

  switch(urdf_collision->geometry->type){

    case urdf::Geometry::BOX:
    {
      const urdf::BoxSharedPtr box =
          urdf::static_pointer_cast<urdf::Box, urdf::Geometry>(urdf_collision->geometry);

      geom_ptr.reset(new CollisionGeometryBox(fabs(box->dim.x),
                                              fabs(box->dim.y),
                                              fabs(box->dim.z),
                                              position, rotation));

      success = true;
      break;
    }

    case urdf::Geometry::CYLINDER:
    {
      const urdf::CylinderSharedPtr cylinder =
          urdf::static_pointer_cast<urdf::Cylinder, urdf::Geometry>(urdf_collision->geometry);

      geom_ptr.reset(new CollisionGeometryCylinder(fabs(cylinder->radius),
                                                   fabs(cylinder->length),
                                                   position, rotation));

      success = true;

      break;
    }

    case urdf::Geometry::SPHERE:
    {
      const urdf::SphereSharedPtr sphere =
          urdf::static_pointer_cast<urdf::Sphere, urdf::Geometry>(urdf_collision->geometry);

      geom_ptr.reset(new CollisionGeometrySphere(fabs(sphere->radius),
                                                   position, rotation));

      success = true;

      break;
    }

    default:
      success = false;
      break;
  }

  return success;
}

bool setComboGeomFromUrdfCollisions(const std::vector<urdf::CollisionSharedPtr>& urdf_cols, std::shared_ptr<CollisionGeometry>& geom_ptr,
                                   const Position& link_position_offset,
                                   const Orientation& link_orientation_offset ){

  bool success = true;

    CollisionGeometryCombo* combo_geom = new CollisionGeometryCombo;
  if (!geom_ptr){
    geom_ptr.reset( combo_geom );
  } else {
    std::unique_ptr<CollisionGeometry> geom_becomes_child(geom_ptr->clone());

    combo_geom->addGeometry(geom_becomes_child);
    geom_ptr.reset( combo_geom );
  }

  for (const urdf::CollisionSharedPtr& urdf_col: urdf_cols){


    std::unique_ptr<CollisionGeometry> new_geom;

    success &= setGeomFromUrdfCollision< std::unique_ptr<CollisionGeometry> >(urdf_col, new_geom, link_position_offset, link_orientation_offset);

    if (new_geom){
      combo_geom->addGeometry(new_geom);
    }
  }

  return success;
}



bool setCollisionsGeometryFromLink(const urdf::Link& urdf_link, std::shared_ptr<CollisionGeometry>& geom_ptr,
                                   const Position& link_position_offset,
                                   const Orientation& link_orientation_offset){

  bool success = false;
  if (!urdf_link.collision){
    // This is a little unintuitive, but if there is no collision geometry, setting geom_ptr is always "successful"
    success = true;
  }
  else if (urdf_link.collision_array.size() > 1 || geom_ptr){
    success = setComboGeomFromUrdfCollisions(urdf_link.collision_array, geom_ptr, link_position_offset, link_orientation_offset);
  }
  else
  {
    success = setGeomFromUrdfCollision<std::shared_ptr<CollisionGeometry> >(urdf_link.collision, geom_ptr, link_position_offset, link_orientation_offset);
  }

  return success;
}

} //  namespace collisions_urdf
