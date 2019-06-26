/*
 * CollisionsMarkerPublisher.cpp
 *
 *  Created on: Aug 8, 2017
 *      Author: Perry Franklin
 */

#include <collisions_visualization/CollisionsMarkerPublisher.hpp>

namespace collisions_visualization {

CollisionsMarkerPublisher::CollisionsMarkerPublisher(const ros::NodeHandle& nh_, const std::string& frame_id):
    col_geom_publisher_(nh_, frame_id){}



void CollisionsMarkerPublisher::publishRigidBody(collisions::CollisionBodyConstPtr rigid_body,
                      const MarkerOptions& marker_options) const{
  col_geom_publisher_.publishRigidBodyMarker(rigid_body, marker_options);
}

void CollisionsMarkerPublisher::publishCollisionGroup(const collisions::CollisionGroup& collision_group,
                                                      const MarkerOptions& marker_options) const{
  const std::vector< collisions::CollisionBodyConstPtr > bodies = collision_group.getBodies();

  for( auto bodyptr: bodies){
    col_geom_publisher_.publishRigidBodyMarker(bodyptr, marker_options);
  }

}

void CollisionsMarkerPublisher::publishCollisionResults(const collisions::CollisionResults& collisions_results,
                                                        const collisions::CollisionGroup& collision_group1,
                                                        const collisions::CollisionGroup& collision_group2,
                                                        const MarkerOptions& marker_options) const{

  const std::vector< collisions::CollisionBodyConstPtr > bodies1 = collision_group1.getBodies();

  MarkerOptions mutable_options = marker_options;

  for( auto bodyptr: bodies1){
    mutable_options.setColor(false);
    for (auto collision_pair: collisions_results.colliding_pairs){
      if (collision_pair.first == bodyptr->getId()){
        mutable_options.setColor(true);
      }
    }
    col_geom_publisher_.publishRigidBodyMarker(bodyptr, mutable_options);
  }

  const std::vector< collisions::CollisionBodyConstPtr > bodies2 = collision_group2.getBodies();

  for( auto bodyptr: bodies2){
    mutable_options.setColor(false);
    for (auto collision_pair: collisions_results.colliding_pairs){
      if (collision_pair.second == bodyptr->getId()){
        mutable_options.setColor(true);
      }
    }
    col_geom_publisher_.publishRigidBodyMarker(bodyptr, mutable_options);
  }

}

} // namespace collisions_visualization
