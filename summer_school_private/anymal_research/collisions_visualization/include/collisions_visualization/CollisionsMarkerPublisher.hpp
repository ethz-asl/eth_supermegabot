/*
 * CollisionsMarkerPublisher.hpp
 *
 *  Created on: Aug 8, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <collisions_visualization/CollisionGeometryMarkerPublisher.hpp>

#include <ros/ros.h>

#include <collisions/CollisionGroup.hpp>
#include <collisions/CollisionResults.hpp>

#include <collisions/CollisionBody.hpp>

namespace collisions_visualization {

class CollisionsMarkerPublisher{

public:
  CollisionsMarkerPublisher(const ros::NodeHandle& nh_, const std::string& frame_id = "world");
  ~CollisionsMarkerPublisher() = default;

  void publishRigidBody(collisions::CollisionBodyConstPtr rigid_body,
                        const MarkerOptions& marker_options_ = MarkerOptions()) const;


  void publishCollisionGroup(const collisions::CollisionGroup& collision_group,
                             const MarkerOptions& marker_options_ = MarkerOptions()) const;

  void publishCollisionResults(const collisions::CollisionResults& collisions_results,
                               const collisions::CollisionGroup& collision_group1,
                               const collisions::CollisionGroup& collision_group2,
                               const MarkerOptions& marker_options_ = MarkerOptions()) const;

private:
  CollisionGeometryMarkerPublisher col_geom_publisher_;

};

} // namespace collisions_visualization
