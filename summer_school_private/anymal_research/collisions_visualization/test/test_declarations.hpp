/*
 * test_declarations.hpp
 *
 *  Created on: June 3, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <kindr/Core>

#include <romo/common/RigidBodyItem.hpp>

#include <romo/common/containers.hpp>

#include <gtest/gtest.h>

namespace collisions_visualization_test {

enum class CoordinateFrame : unsigned int { WORLD = 0, LOCAL };

enum class BodyEnum : unsigned int {
  BODY0 = 0,
  BODY1,
  BODY2,
  BODY3
};

enum class BodyNodeEnum : unsigned int {
  NODE0 = 0,
  NODE1,
  NODE2,
  NODE3
};

enum class BranchEnum : unsigned int {
  ALL = 0
};

template <typename BodyEnum_, typename BodyNodeEnum_, typename BranchEnum_, typename CoordinateFrame_>
class TestRigidBodyBase : public romo::RigidBodyItem<BodyEnum_, BodyNodeEnum_, BranchEnum_,CoordinateFrame_> {
private:
 using Base = romo::RigidBodyItem<BodyEnum_, BodyNodeEnum_, BranchEnum_, CoordinateFrame_>;
 using CoordinateFrame = typename Base::CoordinateFrame;

 public:
 TestRigidBodyBase()
      : inertia_(Eigen::Matrix3d::Identity()) { transform_.setIdentity() ; }
 TestRigidBodyBase(const std::string& name,
                BodyEnum_ body,
                BodyNodeEnum_ bodyNode,
                BranchEnum_ branch)
      : Base(name, body, bodyNode, branch),
        inertia_(Eigen::Matrix3d::Identity()) { transform_.setIdentity() ; }

  // Get the world to body position in world or main body frame
  virtual Eigen::Vector3d getPositionWorldToBody(
      const CoordinateFrame& frame = CoordinateFrame::WORLD) const {
    return getPositionWorldToPointOnBody(Eigen::Vector3d::Zero(), frame);
  }

  // Get the world to point on body position in world or main body frame
  virtual Eigen::Vector3d getPositionWorldToPointOnBody(
      const Eigen::Vector3d& pointOnBody,
      const CoordinateFrame& frame = CoordinateFrame::WORLD) const {
    switch (frame) {
      case (CoordinateFrame::WORLD): {
        return transform_.transform(kindr::HomTransformMatrixD::Position()).vector();
      } break;

      case (CoordinateFrame::LOCAL): {
        return pointOnBody;
      } break;
      default:
        throw std::range_error(
            "[TestRigidBody::getPositionWorldToPointOnBody]: CoordinateFrame "
            "is not supported!");
    }
  }

  // Get the world to body center of mass position in world or main body frame
  virtual Eigen::Vector3d getPositionWorldToBodyCom(
      const CoordinateFrame& frame = CoordinateFrame::WORLD) const {
    return transform_.getPosition().vector();
  }

  // Set the world to body center of mass position in world or main body frame
  virtual bool setPositionWorldToBodyCom(
      const Eigen::Vector3d& centerOfMass,
      const CoordinateFrame& frame = CoordinateFrame::WORLD) const {
    return false;
  }

  // Get the world to body center of mass position in world or main body frame
  virtual Eigen::Vector3d getPositionBodyToBodyCom(
      const CoordinateFrame& frame = CoordinateFrame::WORLD) const {
    return Eigen::Vector3d::Zero();
  }

  // Set the world to body center of mass position in world or main body frame
  virtual bool setPositionBodyToBodyCom(
      const Eigen::Vector3d& centerOfMass,
      const CoordinateFrame& frame = CoordinateFrame::WORLD) const {
    return false;
  }

  // Get the body inertia matrix
  virtual const Eigen::Matrix3d& getInertiaMatrix() const { return inertia_; }

  // Set the body inertia matrix
  virtual bool setInertiaMatrix(const Eigen::Matrix3d& inertiaMatrix) const {
    return false;
  }

  // Get the body mass
  virtual double getMass() const { return 1.0; }

  // Set the body mass
  virtual bool setMass(const double mass) const { return false; }

  // Update the inertia properties
  virtual bool updateInertiaProperties() const { return false; }

  // Get the rotation matrix which rotates a vector from world frame to body
  // frame
  virtual const Eigen::Matrix3d& getOrientationWorldToBody() const {
    return world_to_body_rotation_.toImplementation();

  }

  // Get a unique id for this RigidBody
  virtual unsigned int getId() const{
    return 0;
  }

  // Get isFixed property
  virtual bool getIsFixedBody() const{
    return false;
  }

  void setTransform(const kindr::HomTransformMatrixD& transform){
    transform_ = transform;
    world_to_body_rotation_ = transform.getRotation().inverted();
  }

  const kindr::HomTransformMatrixD& getTransform(){
    return transform_;
  }

 private:
  Eigen::Matrix3d inertia_;

  kindr::HomTransformMatrixD transform_;
  kindr::HomTransformMatrixD::Rotation world_to_body_rotation_;
};

using TestRigidBody = TestRigidBodyBase<BodyEnum, BodyNodeEnum, BranchEnum, CoordinateFrame>;

using TestRigidBodyContainer = romo::RigidBodyShPtrContainer<BodyEnum, BodyNodeEnum, BranchEnum, CoordinateFrame>;

void fillTestBodyContainer(TestRigidBodyContainer& rigidBodyContainer);

} // namespace collisions_fcl_tests
