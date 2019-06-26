/*
 * ColliderManagerBullet.cpp
 *
 *  Created on: Apr 18, 2018
 *      Author: Perry Franklin
 */

#include "collisions_bullet/ColliderManagerBullet.hpp"

#include "collisions_bullet/btCollisionObjectContainer.hpp"
#include "collisions_bullet/conversions.hpp"

#include <exception>

namespace collisions_bullet {

collisions::DistanceResults ColliderManagerBullet::checkDistance(const collisions::CollisionGroup& collision_group1,
                                                                 const collisions::CollisionGroup& collision_group2,
                                                                 const collisions::DistanceOptions& distance_options ){

  std::vector<btCollisionObjectContainer> bullet_group_1;
  for (const collisions::CollisionBodyConstPtr& collisions_body: collision_group1.getBodies()){
    assert(collisions_body && "ColliderManagerBullet::checkDistance: You passed an CollisionBodyConstPtr in the collision_group1 that doesn't exist!");
    bullet_group_1.emplace_back();
    Converter::bulletObjectFromCollisionBody(*collisions_body, 0.0, bullet_group_1.back());
  }

  std::vector<btCollisionObjectContainer> bullet_group_2;
  for (const collisions::CollisionBodyConstPtr& collisions_body: collision_group2.getBodies()){
    assert(collisions_body && "ColliderManagerBullet::checkDistance: You passed an CollisionBodyConstPtr in the collision_group2 that doesn't exist!");
    bullet_group_2.emplace_back();
    Converter::bulletObjectFromCollisionBody(*collisions_body, 0.0, bullet_group_2.back());
  }

  collisions::DistanceResults results;

  collisions::DistanceResult lone_result;
  lone_result.distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < bullet_group_1.size(); ++i){
    for (size_t j = 0; j < bullet_group_2.size(); ++j){

      btCollisionObjectContainer& bullet_object_1 = bullet_group_1[i];
      btCollisionObjectContainer& bullet_object_2 = bullet_group_2[j];

      if (bullet_object_1.isNull()){
        continue;
      }
      if (bullet_object_2.isNull()){
        continue;
      }

      BulletDistanceResult bullet_result = getDistanceBetweenShapes(bullet_object_1.getCollisionObject()->getCollisionShape(),
                                                                    bullet_object_1.getCollisionObject()->getWorldTransform(),
                                                                    bullet_object_2.getCollisionObject()->getCollisionShape(),
                                                                    bullet_object_2.getCollisionObject()->getWorldTransform());

      collisions::DistanceResult collisions_result;

      collisions_result.distance_pair.first = collision_group1.getBodies()[i]->getId();
      collisions_result.distance_pair.second = collision_group2.getBodies()[j]->getId();

      collisions_result.distance = bullet_result.distance;

      collisions_result.nearest_point_1.x() = bullet_result.witnesses[0].x();
      collisions_result.nearest_point_1.y() = bullet_result.witnesses[0].y();
      collisions_result.nearest_point_1.z() = bullet_result.witnesses[0].z();

      collisions_result.nearest_point_2.x() = bullet_result.witnesses[1].x();
      collisions_result.nearest_point_2.y() = bullet_result.witnesses[1].y();
      collisions_result.nearest_point_2.z() = bullet_result.witnesses[1].z();

      if (!distance_options.only_smallest_distance){
        results.getResults().emplace(std::make_pair(collisions_result.distance_pair, collisions_result));
      } else {
        if (collisions_result.distance < lone_result.distance){
          lone_result = collisions_result;
        }
      }

    }
  }
  if (distance_options.only_smallest_distance){
    results.getResults().emplace(std::make_pair(lone_result.distance_pair, lone_result));

  }

  return results;

}


collisions::CollisionResults ColliderManagerBullet::checkCollision(const collisions::CollisionGroup& group1,
                                                                   const collisions::CollisionGroup& group2,
                                                                   const collisions::CollisionOptions& collision_options){
  throw std::runtime_error("Don't call checkCollision on a ColliderManagerBullet, it is not supported");
}


typename ColliderManagerBullet::BulletDistanceResult ColliderManagerBullet::getDistanceBetweenShapes(btCollisionShape* shape_1, btTransform transform_1,
                                                                                                     btCollisionShape* shape_2, btTransform transform_2){

  BulletDistanceResult current_best_result;
  current_best_result.distance = std::numeric_limits<double>::max();

  if (shape_1->isCompound()){
    btCompoundShape* compound_shape = dynamic_cast<btCompoundShape*>(shape_1);
    int num_children = compound_shape->getNumChildShapes();

    for (int i = 0; i < num_children; ++i){

      BulletDistanceResult new_compound_result
                               = getDistanceBetweenShapes(compound_shape->getChildShape(i), transform_1*compound_shape->getChildTransform(i),
                                                          shape_2, transform_2);
      if (new_compound_result.distance < current_best_result.distance){
        current_best_result = new_compound_result;
      }

    }
  } else if (shape_2->isCompound()){
    btCompoundShape* compound_shape = dynamic_cast<btCompoundShape*>(shape_2);
    int num_children = compound_shape->getNumChildShapes();

    current_best_result.distance = std::numeric_limits<double>::max();
    for (int i = 0; i < num_children; ++i){

      BulletDistanceResult new_compound_result
                               = getDistanceBetweenShapes(shape_1, transform_1,
                                                          compound_shape->getChildShape(i), transform_2*compound_shape->getChildTransform(i));

      if (new_compound_result.distance < current_best_result.distance){
        current_best_result = new_compound_result;
      }

    }
  }

  else if (shape_1->isConvex() && shape_2->isConvex()) {

    btConvexShape* convex_1 = dynamic_cast<btConvexShape*>(shape_1);
    btConvexShape* convex_2 = dynamic_cast<btConvexShape*>(shape_2);

    btGjkEpaSolver2::SignedDistance(convex_1, transform_1, convex_2, transform_2, transform_2.getOrigin() - transform_1.getOrigin(), current_best_result);
  } else {
    std::cerr<<"ColliderManagerBullet::getDistanceBetweenShapes only supports convex object or compound objects made of convex objects - you have something different"<<std::endl;
  }

  return current_best_result;


}


}  // namespace collisions_bullet
