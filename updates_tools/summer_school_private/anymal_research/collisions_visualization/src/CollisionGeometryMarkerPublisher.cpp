/*
 * CollisionGeometryMarkerPublisher.cpp
 *
 *  Created on: Aug 3, 2017
 *      Author: Perry Franklin
 */

#include <collisions_visualization/CollisionGeometryMarkerPublisher.hpp>

namespace collisions_visualization{

CollisionGeometryMarkerPublisher::CollisionGeometryMarkerPublisher(const ros::NodeHandle& nh , const std::string& frame_id):
  nh_(nh),
  frame_id_(frame_id){
  marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("collision_geometry", 0);
}

void CollisionGeometryMarkerPublisher::publishRigidBodyMarker(collisions::CollisionBodyConstPtr  rigid_body, const MarkerOptions& options) const{

    visualization_msgs::MarkerArray marker_array = CollisionsGeomMarkerConverter::bodyToMarkerWithId(rigid_body, options);

    publishMarkerArray(marker_array);
  }

void CollisionGeometryMarkerPublisher::publishMarkerArray(const visualization_msgs::MarkerArray& marker_array) const{
  marker_publisher_.publish(changeMarkersFrameId(marker_array));
}

visualization_msgs::MarkerArray CollisionGeometryMarkerPublisher::changeMarkersFrameId(const visualization_msgs::MarkerArray& marker_array) const{

  visualization_msgs::MarkerArray modified_marker_array = marker_array;
  for (visualization_msgs::Marker& marker: modified_marker_array.markers){
    marker.header.frame_id = frame_id_;
  }
  return modified_marker_array;
}


}

