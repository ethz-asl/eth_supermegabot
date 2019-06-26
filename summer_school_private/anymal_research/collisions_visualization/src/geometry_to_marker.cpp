/*
 * geometry_to_marker.cpp
 *
 *  Created on: Aug 3, 2017
 *      Author: Perry Franklin
 */

#include <collisions_visualization/geometry_to_marker.hpp>

#include <ros/console.h>

namespace collisions_visualization {

visualization_msgs::MarkerArray CollisionsGeomMarkerConverter::bodyToMarkerWithId(
    const collisions::CollisionBodyConstPtr& rigid_body, const MarkerOptions& options) {
  uintptr_t id = (uintptr_t) rigid_body.get();
  visualization_msgs::MarkerArray marker_array = bodyToMarker(rigid_body, options);

  for (unsigned int i = 0; i < marker_array.markers.size(); ++i){

    marker_array.markers[i].id = id + i;

  }

  return marker_array;
}

visualization_msgs::MarkerArray CollisionsGeomMarkerConverter::bodyToMarker(
    const collisions::CollisionBodyConstPtr& rigid_body, const MarkerOptions& options) {

  if (!rigid_body->getCollisionGeometry()) {
    ROS_WARN_STREAM("collision_geometry has no geometry");
    return visualization_msgs::MarkerArray();
  }
  visualization_msgs::MarkerArray marker_array = geomToMarker(*(rigid_body->getCollisionGeometry()));

  kindr::HomTransformMatrixD world_to_body_matrix(rigid_body->getPositionInWorld(),
                                                  rigid_body->getOrientationInWorld());

  kindr::HomTransformQuatD world_to_body = kindr::HomTransformQuatD( world_to_body_matrix);

  for (unsigned int i = 0; i < marker_array.markers.size(); ++i){

    visualization_msgs::Marker& marker = marker_array.markers[i];
    kindr::HomTransformQuatD body_to_geom(kindr::Position3D(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z),
                                          kindr::RotationQuaternionD(marker.pose.orientation.w,marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z));

    kindr::HomTransformQuatD final_location = world_to_body*body_to_geom;

    marker.pose.position.x = final_location.getPosition().x();
    marker.pose.position.y = final_location.getPosition().y();
    marker.pose.position.z = final_location.getPosition().z();
    marker.pose.orientation.w = final_location.getRotation().w();
    marker.pose.orientation.x = final_location.getRotation().x();
    marker.pose.orientation.y = final_location.getRotation().y();
    marker.pose.orientation.z = final_location.getRotation().z();

    options.applyOptions(marker);

  }

  return marker_array;
}


visualization_msgs::MarkerArray CollisionsGeomMarkerConverter::geomToMarker(const collisions_geometry::CollisionGeometry& geometry)
{

visualization_msgs::MarkerArray marker_array;

switch (geometry.getType()) {

  case collisions_geometry::CollisionGeometry::GeomType::BOX:
  {
    const collisions_geometry::CollisionGeometryBox& geometry_box = static_cast<const collisions_geometry::CollisionGeometryBox&>(geometry);

    marker_array = boxToMarker(geometry_box);

    break;
  }
  case collisions_geometry::CollisionGeometry::GeomType::CYLINDER:
  {
    const collisions_geometry::CollisionGeometryCylinder& geometry_cylinder = static_cast<const collisions_geometry::CollisionGeometryCylinder&>(geometry);

    marker_array = cylinderToMarker(geometry_cylinder);

    break;
  }
  case collisions_geometry::CollisionGeometry::GeomType::CAPSULE:
  {
    const collisions_geometry::CollisionGeometryCapsule& geometry_capsule = static_cast<const collisions_geometry::CollisionGeometryCapsule&>(geometry);

    marker_array = capsuleToMarker(geometry_capsule);

    break;
  }
  case collisions_geometry::CollisionGeometry::GeomType::COMBO:
  {
    const collisions_geometry::CollisionGeometryCombo& geometry_combo = static_cast<const collisions_geometry::CollisionGeometryCombo&>(geometry);

    marker_array = comboToMarker(geometry_combo);

    break;
  }
  default:

    ROS_WARN("Did not convert collision_geometry to a marker");
}

return marker_array;

}


visualization_msgs::MarkerArray CollisionsGeomMarkerConverter::boxToMarker(const collisions_geometry::CollisionGeometryBox& geometry_box) {

  visualization_msgs::MarkerArray marker_array;

  marker_array.markers.push_back(createEmptyMarker());
  visualization_msgs::Marker& current_marker = marker_array.markers[0];

  current_marker.type = visualization_msgs::Marker::CUBE;
  current_marker.scale.x = geometry_box.getXSize();
  current_marker.scale.y = geometry_box.getYSize();
  current_marker.scale.z = geometry_box.getZSize();

  current_marker.pose.position.x = geometry_box.getPosition().x();
  current_marker.pose.position.y = geometry_box.getPosition().y();
  current_marker.pose.position.z = geometry_box.getPosition().z();

  kindr::RotationQuaternionD quaternion = kindr::RotationQuaternionD(geometry_box.getRotation());

  current_marker.pose.orientation.x = quaternion.x();
  current_marker.pose.orientation.y = quaternion.y();
  current_marker.pose.orientation.z = quaternion.z();
  current_marker.pose.orientation.w = quaternion.w();

  return marker_array;
}


visualization_msgs::MarkerArray CollisionsGeomMarkerConverter::cylinderToMarker(const collisions_geometry::CollisionGeometryCylinder& geometry_cylinder) {

  visualization_msgs::MarkerArray marker_array;

  marker_array.markers.push_back(createEmptyMarker());
  visualization_msgs::Marker& current_marker = marker_array.markers[0];

  current_marker.type = visualization_msgs::Marker::CYLINDER;
  current_marker.scale.x = geometry_cylinder.getRadius()*2.0;
  current_marker.scale.y = geometry_cylinder.getRadius()*2.0;
  current_marker.scale.z = geometry_cylinder.getLength();

  current_marker.pose.position.x = geometry_cylinder.getPosition().x();
  current_marker.pose.position.y = geometry_cylinder.getPosition().y();
  current_marker.pose.position.z = geometry_cylinder.getPosition().z();

  kindr::RotationQuaternionD quaternion = kindr::RotationQuaternionD(geometry_cylinder.getRotation());

  current_marker.pose.orientation.x = quaternion.x();
  current_marker.pose.orientation.y = quaternion.y();
  current_marker.pose.orientation.z = quaternion.z();
  current_marker.pose.orientation.w = quaternion.w();

  return marker_array;
}


visualization_msgs::MarkerArray CollisionsGeomMarkerConverter::capsuleToMarker(const collisions_geometry::CollisionGeometryCapsule& geometry_capsule) {

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.reserve(3);

  marker_array.markers.push_back(createEmptyMarker());
  visualization_msgs::Marker& cylinder_marker = marker_array.markers[0];

  cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
  cylinder_marker.scale.x = geometry_capsule.getRadius()*2.0;
  cylinder_marker.scale.y = geometry_capsule.getRadius()*2.0;
  cylinder_marker.scale.z = geometry_capsule.getLength();

  cylinder_marker.pose.position.x = geometry_capsule.getPosition().x();
  cylinder_marker.pose.position.y = geometry_capsule.getPosition().y();
  cylinder_marker.pose.position.z = geometry_capsule.getPosition().z();

  kindr::RotationQuaternionD quaternion = kindr::RotationQuaternionD(geometry_capsule.getRotation());

  cylinder_marker.pose.orientation.x = quaternion.x();
  cylinder_marker.pose.orientation.y = quaternion.y();
  cylinder_marker.pose.orientation.z = quaternion.z();
  cylinder_marker.pose.orientation.w = quaternion.w();

  // Technically not the z_axis, but the z_axis scaled to point at the location of the "caps"/spheres.
  kindr::Position3D z_axis = geometry_capsule.getLength()/2*quaternion.rotate(kindr::Position3D::UnitZ());

  { // Generates the sphere in the positive z direction.
    marker_array.markers.push_back(createEmptyMarker());
    visualization_msgs::Marker& sphere_marker = marker_array.markers[1];
    sphere_marker.type = visualization_msgs::Marker::SPHERE;

    sphere_marker.pose.position.x = cylinder_marker.pose.position.x + z_axis.x();
    sphere_marker.pose.position.y = cylinder_marker.pose.position.y + z_axis.y();
    sphere_marker.pose.position.z = cylinder_marker.pose.position.z + z_axis.z();

    sphere_marker.scale.x = geometry_capsule.getRadius()*2.0;
    sphere_marker.scale.y = geometry_capsule.getRadius()*2.0;
    sphere_marker.scale.z = geometry_capsule.getRadius()*2.0;

    sphere_marker.pose.orientation.x = quaternion.x();
    sphere_marker.pose.orientation.y = quaternion.y();
    sphere_marker.pose.orientation.z = quaternion.z();
    sphere_marker.pose.orientation.w = quaternion.w();
  }
  { // Generates the sphere in the negative z direction.
    marker_array.markers.push_back(createEmptyMarker());
    visualization_msgs::Marker& sphere_marker = marker_array.markers[2];
    sphere_marker.type = visualization_msgs::Marker::SPHERE;

    sphere_marker.pose.position.x = cylinder_marker.pose.position.x - z_axis.x();
    sphere_marker.pose.position.y = cylinder_marker.pose.position.y - z_axis.y();
    sphere_marker.pose.position.z = cylinder_marker.pose.position.z - z_axis.z();

    sphere_marker.scale.x = geometry_capsule.getRadius()*2.0;
    sphere_marker.scale.y = geometry_capsule.getRadius()*2.0;
    sphere_marker.scale.z = geometry_capsule.getRadius()*2.0;

    sphere_marker.pose.orientation.x = quaternion.x();
    sphere_marker.pose.orientation.y = quaternion.y();
    sphere_marker.pose.orientation.z = quaternion.z();
    sphere_marker.pose.orientation.w = quaternion.w();
  }

  return marker_array;

}


visualization_msgs::MarkerArray CollisionsGeomMarkerConverter::comboToMarker(const collisions_geometry::CollisionGeometryCombo& geometry_combo) {

  visualization_msgs::MarkerArray marker_array;

  const std::vector<std::unique_ptr<collisions_geometry::CollisionGeometry>>& geoms = geometry_combo.getGeometries();
  for (size_t i = 0; i < geoms.size(); ++i ){

    const visualization_msgs::MarkerArray& temp_array = geomToMarker(*geoms[i]);
    marker_array.markers.insert(marker_array.markers.end(), temp_array.markers.begin(), temp_array.markers.end());
  }

  return marker_array;

}


visualization_msgs::Marker CollisionsGeomMarkerConverter::createEmptyMarker() {
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time(0);
  marker.ns = "collisions_visualization";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;

  return marker;
}

} // namespace collisions_visualization
