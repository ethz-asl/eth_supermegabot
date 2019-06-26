/*
 * visual_test.cpp
 *
 *  Created on: Aug 3, 2017
 *      Author: Perry Franklin
 */

#include <collisions_visualization/geometry_to_marker.hpp>
#include <collisions_visualization/CollisionGeometryMarkerPublisher.hpp>
#include <collisions_visualization/CollisionsMarkerPublisher.hpp>
#include <collisions_visualization/MarkerOptions.hpp>

#include <collisions/CollisionGroup.hpp>

#include <kindr/rotations/RotationConversion.hpp>

#include <ros/ros.h>

#include <gtest/gtest.h>

#include "test_declarations.hpp"

namespace collisions_visualization_test{

double random(double min, double max){
  if (min > max){
    double temp_max = max;
    max = min;
    min = temp_max;
  }
  return ((double) std::rand())/((double) RAND_MAX)*(max-min) + min;
}


using namespace collisions_visualization;

TEST(Visualization, CollisionGeometry)
{
  ros::NodeHandle nodeHandle;

  CollisionGeometryMarkerPublisher col_geom_publisher(nodeHandle, "world1");

  TestRigidBodyContainer container;
  fillTestBodyContainer(container);
  {
    kindr::Position3D position( random(-1.0, 0.0), random(-1.0,0.0), random(0.0,1.0) );
    collisions_geometry::CollisionGeometry::Rotation rotation;
    rotation.RotationBase::setRandom();
    container[BodyEnum::BODY0]->collision_geometry.reset(new collisions_geometry::CollisionGeometryBox(1.0, 0.5, 0.5, position,rotation));
  }
  {
    kindr::Position3D position( random(0.0,1.0), random(-1.0,0.0), random(0.0,1.0) );
    collisions_geometry::CollisionGeometry::Rotation rotation;
    rotation.RotationBase::setRandom();
    container[BodyEnum::BODY1]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCylinder(0.5, 1.0, position,rotation));
  }
  {
    kindr::Position3D position( random(-0.0,-1.0), random(1.0,0.0), random(1.0,0) );
    collisions_geometry::CollisionGeometry::Rotation rotation;
    std::static_pointer_cast<TestRigidBody>(container[BodyEnum::BODY2])->setTransform(kindr::HomTransformMatrixD(position, rotation));
    container[BodyEnum::BODY2]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCapsule(0.5, 1.0, kindr::Position3D(),collisions_geometry::CollisionGeometry::Rotation()));

    position.z() = position.z()+1.0;
    rotation.RotationBase::setRandom();
    container[BodyEnum::BODY3]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCapsule(0.5, 1.0, kindr::Position3D(),collisions_geometry::CollisionGeometry::Rotation()));
    std::static_pointer_cast<TestRigidBody>(container[BodyEnum::BODY3])->setTransform(kindr::HomTransformMatrixD(position, rotation));
  }

  // Lets all the ros and rviz stuff start up.
  ros::Duration(1.0).sleep();

  col_geom_publisher.publishRigidBodyMarker<CoordinateFrame>(container[BodyEnum::BODY0]);

  MarkerOptions options(1.0,1.0,1.0,0.8);
  col_geom_publisher.publishRigidBodyMarker<CoordinateFrame>(container[BodyEnum::BODY1], options);

  MarkerOptions options1(MarkerOptions::ColorType::RED,0.3);
  col_geom_publisher.publishRigidBodyMarker<CoordinateFrame>(container[BodyEnum::BODY2], options1);

  MarkerOptions options2(MarkerOptions::ColorType::BLUE,0.5);
  col_geom_publisher.publishRigidBodyMarker<CoordinateFrame>(container[BodyEnum::BODY3], options2);

}

TEST(Visualization, Collisions)
{
  ros::NodeHandle nodeHandle;

  CollisionsMarkerPublisher col_publisher(nodeHandle, "world2");

  TestRigidBodyContainer container;
  fillTestBodyContainer(container);
  {
    kindr::Position3D position( random(-1.0, 0.0), random(-1.0,0.0), random(0.0,1.0) );
    collisions_geometry::CollisionGeometry::Rotation rotation;
    rotation.RotationBase::setRandom();
    container[BodyEnum::BODY0]->collision_geometry.reset(new collisions_geometry::CollisionGeometryBox(1.0, 0.5, 0.5, position,rotation));
  }
  {
    kindr::Position3D position( random(0.0,1.0), random(-1.0,0.0), random(0.0,1.0) );
    collisions_geometry::CollisionGeometry::Rotation rotation;
    rotation.RotationBase::setRandom();
    container[BodyEnum::BODY1]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCylinder(0.5, 1.0, position,rotation));
  }
  {
    kindr::Position3D position( random(-0.0,-1.0), random(1.0,0.0), random(1.0,0) );
    collisions_geometry::CollisionGeometry::Rotation rotation;
    rotation.RotationBase::setRandom();
    container[BodyEnum::BODY2]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCapsule(0.5, 1.0, position,rotation));
  }

  // Lets all the ros and rviz stuff start up.
  ros::Duration(1.0).sleep();

  col_publisher.publishRigidBody<CoordinateFrame>(container[BodyEnum::BODY0]);

  collisions::CollisionGroup<BodyEnum, BodyNodeEnum, BranchEnum, CoordinateFrame> group({BodyEnum::BODY1, BodyEnum::BODY2}, container);

  MarkerOptions options(1.0,1.0,1.0,0.8);
  col_publisher.publishCollisionGroup(group, options);

}

TEST(Visualization, CollisionsResults)
{
  ros::NodeHandle nodeHandle;

  CollisionsMarkerPublisher col_publisher(nodeHandle, "world3");

  TestRigidBodyContainer container;
  fillTestBodyContainer(container);
  {
    kindr::Position3D position( -1.5, 1.5, 0.2 );
    kindr::EulerAnglesXyzD rotation( 0.0, 0.5, 0.0 );
    container[BodyEnum::BODY0]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCapsule(0.5, 1.0, kindr::Position3D(),collisions_geometry::CollisionGeometry::Rotation()));
    std::static_pointer_cast<TestRigidBody>(container[BodyEnum::BODY0])->setTransform(kindr::HomTransformMatrixD(position, kindr::RotationMatrixD(rotation)));
  }
  {
    kindr::Position3D position( -1.5, 1.5, 1.2 );
    kindr::EulerAnglesXyzD rotation( 0.0, -0.5, 0.0 );
    container[BodyEnum::BODY1]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCapsule(0.5, 1.0, kindr::Position3D(),collisions_geometry::CollisionGeometry::Rotation()));
    std::static_pointer_cast<TestRigidBody>(container[BodyEnum::BODY1])->setTransform(kindr::HomTransformMatrixD(position, kindr::RotationMatrixD(rotation)));
  }
  {
    kindr::Position3D position( -1.8, 1.5, -0.2 );
    kindr::EulerAnglesXyzD rotation( 0.0, 0.5, 0.0 );
    container[BodyEnum::BODY2]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCapsule(0.5, 1.0, kindr::Position3D(),collisions_geometry::CollisionGeometry::Rotation()));
    std::static_pointer_cast<TestRigidBody>(container[BodyEnum::BODY2])->setTransform(kindr::HomTransformMatrixD(position, kindr::RotationMatrixD(rotation)));
  }
  {
    kindr::Position3D position( -1.8, 1.5, -1.2 );
    kindr::EulerAnglesXyzD rotation( 0.0, -0.5, 0.0 );
    container[BodyEnum::BODY3]->collision_geometry.reset(new collisions_geometry::CollisionGeometryCapsule(0.5, 1.0, kindr::Position3D(),collisions_geometry::CollisionGeometry::Rotation()));
    std::static_pointer_cast<TestRigidBody>(container[BodyEnum::BODY3])->setTransform(kindr::HomTransformMatrixD(position, kindr::RotationMatrixD(rotation)));
  }

  // Lets all the ros and rviz stuff start up.
  ros::Duration(1.0).sleep();

  collisions::CollisionGroup<BodyEnum, BodyNodeEnum, BranchEnum, CoordinateFrame> group1({BodyEnum::BODY0, BodyEnum::BODY1}, container);

  collisions::CollisionGroup<BodyEnum, BodyNodeEnum, BranchEnum, CoordinateFrame> group2({BodyEnum::BODY2, BodyEnum::BODY3}, container);

  collisions::CollisionResults< CoordinateFrame, collisions::CollisionGroup<BodyEnum, BodyNodeEnum, BranchEnum, CoordinateFrame>,
                                                 collisions::CollisionGroup<BodyEnum, BodyNodeEnum, BranchEnum, CoordinateFrame> > results;

  results.colliding_pairs.push_back(std::pair<BodyEnum, BodyEnum>(BodyEnum::BODY0,BodyEnum::BODY2));

  col_publisher.publishCollisionResults(results, group1, group2);


}

TEST(Visualization, Wait){

  //   Arbitrary wait for visualization.
    ros::Duration(30.0).sleep();

}

}


