/*
 * geometry_to_marker.hpp
 *
 *  Created on: Aug 3, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <collisions_geometry/CollisionGeometry.hpp>
#include <collisions_geometry/CollisionGeometryBox.hpp>
#include <collisions_geometry/CollisionGeometryCylinder.hpp>
#include <collisions_geometry/CollisionGeometryCapsule.hpp>
#include <collisions_geometry/CollisionGeometryCombo.hpp>

#include <collisions_visualization/MarkerOptions.hpp>

#include <collisions/CollisionBody.hpp>

#include <visualization_msgs/MarkerArray.h>

namespace collisions_visualization{

class CollisionsGeomMarkerConverter{

public:

  CollisionsGeomMarkerConverter() = delete;
  ~CollisionsGeomMarkerConverter() = delete;

  static visualization_msgs::MarkerArray bodyToMarkerWithId(const collisions::CollisionBodyConstPtr&  rigid_body, const MarkerOptions& options = MarkerOptions());

  static visualization_msgs::MarkerArray bodyToMarker(const collisions::CollisionBodyConstPtr&  rigid_body, const MarkerOptions& options = MarkerOptions());

  static visualization_msgs::MarkerArray geomToMarker(const collisions_geometry::CollisionGeometry& geometry);

  static visualization_msgs::MarkerArray boxToMarker(const collisions_geometry::CollisionGeometryBox& geometry_box);

  static visualization_msgs::MarkerArray cylinderToMarker(const collisions_geometry::CollisionGeometryCylinder& geometry_cylinder);

  static visualization_msgs::MarkerArray capsuleToMarker(const collisions_geometry::CollisionGeometryCapsule& geometry_capsule);

  static visualization_msgs::MarkerArray comboToMarker(const collisions_geometry::CollisionGeometryCombo& geometry_combo);

private:

  static visualization_msgs::Marker createEmptyMarker();

};

} // namespace collisions_visualization
