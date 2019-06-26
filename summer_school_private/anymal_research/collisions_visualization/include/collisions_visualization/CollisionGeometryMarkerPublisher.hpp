/*
 * collision_geometry_marker_publisher.hpp
 *
 *  Created on: Aug 3, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <ros/ros.h>

#include <collisions_visualization/geometry_to_marker.hpp>
#include <collisions_visualization/MarkerOptions.hpp>

#include <collisions/CollisionBody.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace collisions_visualization {

class CollisionGeometryMarkerPublisher{

public:

  CollisionGeometryMarkerPublisher(const ros::NodeHandle& nh, const std::string& frame_id = "world" );
  ~CollisionGeometryMarkerPublisher() = default;

  void publishRigidBodyMarker(collisions::CollisionBodyConstPtr  rigid_body, const MarkerOptions& options = MarkerOptions()) const;

private:

  void publishMarkerArray(const visualization_msgs::MarkerArray& marker_array) const;

  visualization_msgs::MarkerArray changeMarkersFrameId(const visualization_msgs::MarkerArray& marker_array) const;

  ros::NodeHandle nh_;
  ros::Publisher marker_publisher_;
  std::string frame_id_;

};

} // namespace collisions_visualization
