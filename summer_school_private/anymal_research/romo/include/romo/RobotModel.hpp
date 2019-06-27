/*
 * RobotModel.hpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include <romo/common/containers.hpp>
#include <romo/common/phys_typedefs.hpp>
#include "romo/common/RigidBody.hpp"
#include "romo/common/Contact.hpp"
#include "romo/common/RobotDescription.hpp"
#include "romo/LimitsInterface.hpp"

#include <robot_utils/math/LinearAlgebra.hpp>

#include <Eigen/Core>

#include <memory>

namespace romo {

template<typename ConcreteDescription_, typename RobotState_>
class RobotModel {
 public:
  using RD = romo::RobotDescription<ConcreteDescription_>;
  using BodyEnum            = typename RD::BodyEnum;
  using BodyNodeEnum        = typename RD::BodyNodeEnum;
  using ContactEnum         = typename RD::ContactEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;
  using BranchEnum          = typename RD::BranchEnum;
  using LimbEnum            = typename RD::LimbEnum;
  using RobotState          = RobotState_;
  using JointPositions      = typename RobotState::JointPositions;
  using JointVelocities     = typename RobotState::JointVelocities;
  using LimitsType          = LimitsInterface<ConcreteDescription_, RobotState_>;

 public:
  using JacobianSpatial = Eigen::Matrix<double, 6, RD::getNumDof()>;
  using JacobianTranslation = Eigen::Matrix<double, 3, RD::getNumDof()>;
  using JacobianRotation = Eigen::Matrix<double, 3, RD::getNumDof()>;

 protected:
  RigidBodyShPtrContainer<ConcreteDescription_> bodyContainer_;
  RigidBodyShPtrBranchNodeContainer<ConcreteDescription_> bodyBranchNodeContainer_;
  ContactShPtrContainer<ConcreteDescription_> contactContainer_;
  std::unique_ptr<LimitsType> limitsPtr_;

 public:
  explicit RobotModel() = default;
  virtual ~RobotModel() = default;

  RobotModel(const RobotModel&) = delete;
  RobotModel& operator=(const RobotModel&) = delete;

  const RigidBodyShPtrContainer<ConcreteDescription_>& getBodyContainer() const { return bodyContainer_; }
  const RigidBodyShPtrBranchNodeContainer<ConcreteDescription_>& getBodyBranchNodeContainer() const { return bodyBranchNodeContainer_; }
  const ContactShPtrContainer<ConcreteDescription_>& getContactContainer() const { return contactContainer_; }

  virtual unsigned int getDofCount() const = 0;

  // Extended state
  virtual void setState(
      const RobotState& state,
      bool updatePosition = true,
      bool updateVelocity = false,
      bool updateAcceleration = false) = 0;

  virtual const RobotState& getState() const = 0;

  virtual void updateKinematics(
      bool updatePosition = true,
      bool updateVelocity = false,
      bool updateAcceleration = false) = 0;

  LimitsType* getLimits() { return limitsPtr_.get(); }
  const LimitsType* getLimits() const { return limitsPtr_.get(); }

  /*! Get the position vector of a body w.r.t. the world frame expressed in a given frame.
   * @param position    the output of the function
   * @param bodyEnum    enum representing a body of the kinematic chain
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionWorldToBody(
      Eigen::Vector3d& position,
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of a body w.r.t. the world frame expressed in a given frame and return a copy of it.
   * @param bodyEnum    the enum that represents the body
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionWorldToBody(
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const = 0;

  /*! Get the position vector of a body w.r.t. the world frame expressed in a given frame.
   * @param position    the output of the function
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionWorldToBody(
      Eigen::Vector3d& position,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Get the position vector of a body w.r.t. the world frame expressed in a given frame.
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionWorldToBody(
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Get the position vector of a body w.r.t. the world frame expressed in a given frame.
   * @param position    the output of the function
   * @param positionBodyToPointOnBodyInBodyFrame   point on body
   * @param bodyEnum    enum representing a body of the kinematic chain
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionWorldToPointOnBody(
      Eigen::Vector3d& position,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of a body w.r.t. the world frame expressed in a given frame and return a copy of it.
   * @param bodyEnum    the enum that represents the body
   * @param positionBodyToPointOnBodyInBodyFrame   point on body
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionWorldToPointOnBody(
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const = 0;

  /*! Get the position vector of a body w.r.t. the world frame expressed in a given frame.
   * @param position    the output of the function
   * @param positionBodyToPointOnBodyInBodyFrame   point on body
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionWorldToPointOnBody(
      Eigen::Vector3d& position,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Get the position vector of a body w.r.t. the world frame expressed in a given frame.
   * @param positionBodyToPointOnBodyInBodyFrame   point on body
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionWorldToPointOnBody(
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Get the position vector of the center of mass of a body w.r.t. the world frame expressed in a given frame.
   * @param position    the output of the function
   * @param bodyEnum    enum representing a body of the kinematic chain
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionWorldToBodyCom(
      Eigen::Vector3d& position,
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const = 0;

  /*! Get the position vector of the center of mass of a body w.r.t. the world frame expressed in a given frame.
   * @param bodyEnum    the enum that represents the body
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionWorldToBodyCom(
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const = 0;

  /*! Get the position vector of the center of mass of a body w.r.t. the world frame expressed in a given frame.
   * @param position    the output of the function
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionWorldToBodyCom(
      Eigen::Vector3d& position,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Get the position vector of the center of mass of a body w.r.t. the world frame expressed in a given frame.
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionWorldToBodyCom(
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of a body w.r.t. the world frame expressed in a given frame and return a copy of it.
   * @param position    the output of the function
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionBodyToBody(
      Eigen::Vector3d& position,
      BodyEnum fromBody,
      BodyEnum toBody,
      CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of a body w.r.t. the world frame expressed in a given frame and return a copy of it.
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionBodyToBody(
      BodyEnum fromBody,
      BodyEnum toBody,
      CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of a body w.r.t. another body expressed in a given frame.
   * @param position    the output of the function
   * @param fromBody    enum representing the body in the kinematic chain. This is the origin of the position vector
   * @param toBranch    enum representing a branch in the kinematic chain. This is the target of the position vector
   * @param toNode      enum representing a node in the branch. This is the target of the position vector
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionBodyToBody(
      Eigen::Vector3d& position,
      BodyEnum fromBody,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of a body w.r.t. another body expressed in a given frame and return a copy of it.
   * @param fromBody    enum representing the body in the kinematic chain. This is the origin of the position vector
   * @param toBranch    enum representing a branch in the kinematic chain. This is the target of the position vector
   * @param toNode      enum representing a node in the branch. This is the target of the position vector
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionBodyToBody(
      BodyEnum fromBody,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of a body w.r.t. another body expressed in a given frame.
   * @param position    the output of the function
   * @param fromBranch  enum representing a branch in the kinematic chain. This is the origin of the position vector
   * @param fromNode    enum representing a node in the branch. This is the origin of the position vector
   * @param toBranch    enum representing a branch in the kinematic chain. This is the target of the position vector
   * @param toNode      enum representing a node in the branch. This is the target of the position vector
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true if successfull
   */
  virtual bool getPositionBodyToBody(
      Eigen::Vector3d& position,
      BranchEnum fromBranch,
      BodyNodeEnum fromNode,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of a body w.r.t. another body expressed in a given frame.
   * @param position    the output of the function
   * @param fromBranch  enum representing a branch in the kinematic chain. This is the origin of the position vector
   * @param fromNode    enum representing a node in the branch. This is the origin of the position vector
   * @param toBranch    enum representing a branch in the kinematic chain. This is the target of the position vector
   * @param toNode      enum representing a node in the branch. This is the target of the position vector
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionBodyToBody(
      BranchEnum fromBranch,
      BodyNodeEnum fromNode,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of the center of mass of a body w.r.t. another body expressed in a given frame.
   * @param position    the output of the function
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionBodyToBodyCom(
    Eigen::Vector3d& position,
    BodyEnum fromBody,
    BodyEnum toBody,
    CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of the center of mass of a body w.r.t. another body expressed in a given frame.
   * @param branch      enum representing a branch of the kinematic chain
   * @param node        enum representing a node in the branch
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionBodyToBodyCom(
    BodyEnum fromBody,
    BodyEnum toBody,
    CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of the center of mass of a body w.r.t. another body expressed in a given frame.
   * @param position    the output of the function
   * @param fromBody    enum representing the body in the kinematic chain. This is the origin of the position vector
   * @param toBranch    enum representing a branch in the kinematic chain. This is the target of the position vector
   * @param toNode      enum representing a node in the branch. This is the target of the position vector
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true
   */
  virtual bool getPositionBodyToBodyCom(
    Eigen::Vector3d& position,
    BodyEnum fromBody,
    BranchEnum toBranch,
    BodyNodeEnum toNode,
    CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of the center of mass of a body w.r.t. another body expressed in a given frame.
   * @param fromBody    enum representing the body in the kinematic chain. This is the origin of the position vector
   * @param toBranch    enum representing a branch in the kinematic chain. This is the target of the position vector
   * @param toNode      enum representing a node in the branch. This is the target of the position vector
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionBodyToBodyCom(
    BodyEnum fromBody,
    BranchEnum toBranch,
    BodyNodeEnum toNode,
    CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of the center of mass of a body w.r.t. another body expressed in a given frame.
   * @param position    the output of the function
   * @param fromBranch  enum representing a branch in the kinematic chain. This is the origin of the position vector
   * @param fromNode    enum representing a node in the branch. This is the origin of the position vector
   * @param toBranch    enum representing a branch in the kinematic chain. This is the target of the position vector
   * @param toNode      enum representing a node in the branch. This is the target of the position vector
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns true if successfull
   */
  virtual bool getPositionBodyToBodyCom(
    Eigen::Vector3d& position,
    BranchEnum fromBranch,
    BodyNodeEnum fromNode,
    BranchEnum toBranch,
    BodyNodeEnum toNode,
    CoordinateFrameEnum frame) const = 0;

  /*! Compute the position vector of the center of mass of a body w.r.t. another body expressed in a given frame.
   * @param position    the output of the function
   * @param fromBranch  enum representing a branch in the kinematic chain. This is the origin of the position vector
   * @param fromNode    enum representing a node in the branch. This is the origin of the position vector
   * @param toBranch    enum representing a branch in the kinematic chain. This is the target of the position vector
   * @param toNode      enum representing a node in the branch. This is the target of the position vector
   * @param frame       the frame in which the position vector should be expressed. Default is CoordinateFrameEnum::WORLD.
   * @returns the position of the body
   */
  virtual Eigen::Vector3d getPositionBodyToBodyCom(
    BranchEnum fromBranch,
    BodyNodeEnum fromNode,
    BranchEnum toBranch,
    BodyNodeEnum toNode,
    CoordinateFrameEnum frame) const = 0;

  /*! Set the position vector of the center of mass of a movable body w.r.t. this body expressed in a given frame.
   *  This body cannot be a fixed body. Upon calling, any fixed children of the movable body will have their mass properties set to zero.
   * @param position           the new position of the individual body CoM
   * @param body               enum representing a body in the kinematic chain. This is the origin of the position vector.
   * @param frame              the frame in which the position vector should be expressed.
   * @param updateBodyInertia  whether to compute the new movable body inertia properties
   * @returns true if successful, false otherwise
   */
  virtual bool setPositionBodyToBodyCom(
      const Eigen::Vector3d& position,
      BodyEnum body,
      CoordinateFrameEnum frame,
      bool updateBodyInertia = true) const = 0;

  /*! Set the position vector of the center of mass of an individual body w.r.t. this body expressed in a given frame.
   *  This body can be either a movable or fixed body. Any fixed children of this body are not affected.
   * @param position           the new position of the individual body CoM
   * @param body               enum representing a body in the kinematic chain. This is the origin of the position vector.
   * @param frame              the frame in which the position vector should be expressed.
   * @param updateBodyInertia  whether to compute the new movable body inertia properties
   * @returns true if successful, false otherwise
   */
  virtual bool setPositionBodyToIndividualBodyCom(
      const Eigen::Vector3d& position,
      BodyEnum body,
      CoordinateFrameEnum frame,
      bool updateBodyInertia = true) const = 0;

  /*! Get the total center of mass of a branch
   * @param branch    the target branch
   * @returns the total mass of a branch
   */
  virtual Eigen::Vector3d getPositionBaseToLimbComInBaseFrame(BranchEnum branch) const = 0;

  /*! @returns the position of the center of mass of the robot with respect to the origin of the world.
   * @param frame   coordinate frame the position is expressed in
   */
  virtual void getPositionWorldToCom(
      Eigen::VectorXd& positionWorldToCom,
      CoordinateFrameEnum frame) const = 0;

  /*! @returns the position of the center of mass of the robot with respect to the origin of the world.
   * @param frame   coordinate frame the position is expressed in
   */
  virtual Eigen::Vector3d getPositionWorldToCom(
      CoordinateFrameEnum frame) const = 0;
  /************************/

  /************************
   * Get orientation methods *
   ************************/

  /*! Get the rotation matrix that projects the components of a 3d vector expressed in world frame to those
   * of a 3d vector expressed in a body frame.
   * @param body    the target body enum
   * @returns the rotation matrix
   */
  virtual const Eigen::Matrix3d& getOrientationWorldToBody(BodyEnum bodyEnum) const = 0;

  /*! Get the rotation matrix that projects the components of a 3d vector expressed in world frame to those
   * of a 3d vector expressed in a body frame.
   * @param branch    the target branch enum
   * @param bodyNode  the target node enum
   * @returns the rotation matrix
   */
  virtual const Eigen::Matrix3d& getOrientationWorldToBody(
      BranchEnum branch,
      BodyNodeEnum bodyNode) const = 0;

  /*! Get the rotation matrix that projects the components of a 3d vector expressed in a body frame to those
   * of a 3d vector expressed in a different body frame.
   * @param fromBody  the end body enum
   * @param toBody    the start body enum
   * @returns the rotation matrix
   */
  virtual const Eigen::Matrix3d getOrientationBodyToBody(
      BodyEnum fromBody,
      BodyEnum toBody) const = 0;

  /*! Get the rotation matrix that projects the components of a 3d vector expressed in a body frame to those
   * of a 3d vector expressed in a different body frame.
   * @param fromBranch    the end branch enum
   * @param fromBodyNode  the end node enum
   * @param toBranch      the start branch enum
   * @param toBodyNode    the start node enum
   * @returns the rotation matrix
   */
  virtual const Eigen::Matrix3d getOrientationBodyToBody(
      BranchEnum fromBranch,
      BodyNodeEnum fromBodyNode,
      BranchEnum toBranch,
      BodyNodeEnum toBodyNode) const = 0;


  /************************
   * Get Jacobian methods *
   ************************/
  /*! Gets the 6xDOF Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian is in the form J = [Jr;
   *                                  Jp];
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @param branch      branch of the body belongs to
   * @param node        node of the body
   * @param frame       coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianSpatialWorldToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 6xDOF Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian is in the form J = [Jr;
   *                                  Jp];
   * The Jacobian needs to have the correct size.
   * @param jacobian                               output
   * @param positionBodyToPointOnBodyInBodyFrame   point on body for which jacobian is calculated
   * @param branch                                 branch of the body belongs to
   * @param node                                   node of the body
   * @param frame                                  coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianSpatialWorldToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 6xDOF time derivative of the Jacobian matrix.
     * Note that this method only changes the non-zero entries of the jacobian matrix.
     * The Jacobian is in the form dJ = [dJr;
     *                                   dJp];
     * The Jacobian needs to have the correct size.
     * @param jacobian    output
     * @param branch      branch of the body belongs to
     * @param node        node of the body
     * @param frame       coordinate frame the Jacobian is expressed in
     * @returns  true
     */
  virtual bool getJacobianSpatialTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianSpatialTimeDerivativeWorldToBody,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 6xDOF time derivative of the Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian is in the form dJ = [dJr;
   *                                   dJp];
   * The Jacobian needs to have the correct size.
   * @param jacobianDerivative    output
   * @param jacobian              Jacobian matrix
   * @param branch                branch of the body belongs to
   * @param node                  node of the body
   * @param frame                 coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianSpatialTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianTimeDerivativeWorldToBody,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 6xDOF Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian is in the form J = [Jr;
   *                                  Jp];
   * The Jacobian needs to have the correct size.
   * @param jacobian                               output
   * @param positionBodyToPointOnBodyInBodyFrame   point on body for which jacobian is calculated
   * @param branch                                 branch of the body belongs to
   * @param node                                   node of the body
   * @param frame                                  coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianSpatialTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianTimeDerivativeWorldToBody,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 6xDOF Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian is in the form J = [Jr;
   *                                  Jp];
   * The Jacobian needs to have the correct size.
   * @param jacobian                               output
   * @param jacobianSpatialWorldToPointOnBody      spatial jacobian of point on body
   * @param positionBodyToPointOnBodyInBodyFrame   point on body for which jacobian is calculated
   * @param branch                                 branch of the body belongs to
   * @param node                                   node of the body
   * @param frame                                  coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianSpatialTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianTimeDerivativeWorldToBody,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF translational Jacobian matrix, which is the
   * derivative of the position from world to the body w.r.t. the
   * generalized coordinates (Quaternion).
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @param branch      branch of the body belongs to
   * @param node        node of the body
   * @param frame       coordinate frame the Jacobian is expressed in
   * @returns  true
   *
   * @note This method is unit tested.
   */
  virtual bool getJacobianTranslationWorldToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF translational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian                                  output
   * @param positionBodyToPointOnBodyInBodyFrame      point on the body expressed in body frame
   * @param branch                                    branch of the body belongs to
   * @param node                                      node of the body
   * @param frame                                     coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianTranslationWorldToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF translational Jacobian matrix from world to the
   * center of mass of a given body.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @param branch      branch of the body belongs to
   * @param node        node of the body
   * @param frame       coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianTranslationWorldToBodyCom(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF translational Jacobian matrix, which is the
   * derivative of the position from world to the center of mass of the robot
   * w.r.t. the generalized coordinates (Quaternion).
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @returns  true
   *
   * @note This method is unit tested.
   */
  virtual bool getJacobianTranslationWorldToCom(
      Eigen::MatrixXd& jacobian,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF time derivative of the translational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian          output
   * @param spatialJacobian   the jacobian used to compute its time derivative
   * @param branch            branch of the body belongs to
   * @param node              node of the body
   * @param frame             coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianTranslationTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianTranslationTimeDerivative,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF time derivative of the translational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @param branch      branch of the body belongs to
   * @param node        node of the body
   * @param frame       coordinate frame the Jacobian is expressed in
   * @returns  true
   *
   * @note This method is unit tested.
   */
  virtual bool getJacobianTranslationTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianTranslationTimeDerivative,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF time derivative of the translational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian          output
   * @param spatialJacobian   the jacobian used to compute its time derivative
   * @param positionBodyToPointOnBodyInBodyFrame   point On body for which to calculate the jacobian
   * @param branch            branch of the body belongs to
   * @param node              node of the body
   * @param frame             coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianTranslationTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianTranslationTimeDerivative,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF time derivative of the translational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian                               output
   * @param positionBodyToPointOnBodyInBodyFrame   point On body for which to calculate the jacobian
   * @param branch                                 branch of the body belongs to
   * @param node                                   node of the body
   * @param frame                                  coordinate frame the Jacobian is expressed in
   * @returns  true
   *
   * @note This method is unit tested.
   */
  virtual bool getJacobianTranslationTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianTranslationTimeDerivative,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF translational Jacobian time derivative matrix, which is the
   * derivative of the position from world to the center of mass of the robot
   * w.r.t. the generalized coordinates (Quaternion).
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @returns  true
   *
   * @note This method is unit tested.
   */
  virtual bool getJacobianTranslationTimeDerivativeWorldToCom(
      Eigen::MatrixXd& jacobianTranslationTimeDerivativeWorldToCom,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF rotational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @param branch      branch of the body belongs to
   * @param node        node of the body
   * @param frame       coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianRotationWorldToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF rotational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @param point       a point on the body expressed in body frame
   * @param branch      branch of the body belongs to
   * @param node        node of the body
   * @param frame       coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianRotationWorldToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF time derivative of the rotational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @param spatialJacobian   spatial jacobian
   * @param branch      branch of the body belongs to
   * @param node        node of the body
   * @param frame       coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianRotationTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianRotationTimeDerivative,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF time derivative of the rotational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian    output
   * @param branch      branch of the body belongs to
   * @param node        node of the body
   * @param frame       coordinate frame the Jacobian is expressed in
   * @returns  true
   *
   * @note This method is unit tested.
   */
  virtual bool getJacobianRotationTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianRotationTimeDerivative,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF time derivative of the rotational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian          output
   * @param spatialJacobian   output
   * @param positionBodyToPointOnBodyInBodyFrame   point On body for which to calculate the jacobian
   * @param branch            branch of the body belongs to
   * @param node              node of the body
   * @param frame             coordinate frame the Jacobian is expressed in
   * @returns  true
   */
  virtual bool getJacobianRotationTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianRotationTimeDerivative,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF time derivative of the rotational Jacobian matrix.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian                               output
   * @param positionBodyToPointOnBodyInBodyFrame   point On body for which to calculate the jacobian
   * @param branch                                 branch of the body belongs to
   * @param node                                   node of the body
   * @param frame                                  coordinate frame the Jacobian is expressed in
   * @returns  true
   *
   * @note This method is unit tested.
   */
  virtual bool getJacobianRotationTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianRotationTimeDerivative,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /************************
   * Get Floating Base Jacobian methods *
   ************************/

  /*! Gets the 3xLimbDof translational Jacobian matrix, which is the
   * derivative of the position from main body to end point of one of the limbs
   * w.r.t. the generalized coordinates (Quaternion).
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian                                  output
   * @param branch                                    branch of the body belongs to
   * @param node                                      node of the body
   * @param frame                                     coordinate frame the Jacobian is expressed in
   * @returns  true
   * @note This method is unit tested.
   */
  virtual bool getJacobianTranslationFloatingBaseToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xLimbDof translational Jacobian matrix, which is the
   * derivative of the position from main body to end point of one of the limbs
   * w.r.t. the generalized coordinates (Quaternion).
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian                                  output
   * @param positionBodyToPointOnBodyInBodyFrame      point On body for which to calculate the jacobian
   * @param branch                                    branch of the body belongs to
   * @param node                                      node of the body
   * @param frame                                     coordinate frame the Jacobian is expressed in
   * @returns  true
   * @note This method is unit tested.
   */
  virtual bool getJacobianTranslationFloatingBaseToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xLimbDOF translational Jacobian matrix, which is the
   * derivative of the position from main body to the center of mass of the links of one of the limbs
   * w.r.t. the generalized coordinates (Quaternion).
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian                                  output
   * @param branch                                    branch of the body belongs to
   * @param node                                      node of the body
   * @param frame                                     coordinate frame the Jacobian is expressed in
   * @returns  true
   * @note This method is unit tested.
   */
  virtual bool getJacobianTranslationFloatingBaseToBodyCom(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the joint rotational Jacobian matrix, which is the mapping from limb joint space velocities
   * to task-space angular velocities.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian                                  output
   * @param branch                                    branch of the body belongs to
   * @param node                                      node of the body
   * @param frame                                     coordinate frame the Jacobian is expressed in
   * @returns  true
   * @note This method is unit tested.
   */
  virtual bool getJacobianRotationFloatingBaseToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the joint rotational Jacobian matrix, which is the mapping from limb joint space velocities
   * to task-space angular velocities.
   * Note that this method only changes the non-zero entries of the jacobian matrix.
   * The Jacobian needs to have the correct size.
   * @param jacobian                                  output
   * @param positionBodyToPointOnBodyInBodyFrame      point On body for which to calculate the jacobian
   * @param branch                                    branch of the body belongs to
   * @param node                                      node of the body
   * @param frame                                     coordinate frame the Jacobian is expressed in
   * @returns  true
   * @note This method is unit tested.
   */
  virtual bool getJacobianRotationFloatingBaseToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;
  /************************/

  /************************
   * Get manipulability measures *
   ************************/
  virtual double getManipulabilityMeasure(
      BranchEnum branch,
      BodyNodeEnum bodyNode) const = 0;

  virtual void getManipulabilityMeasureGradientWorldToBody(
      Eigen::VectorXd& manipulabilityMeasureGradient,
      BranchEnum branch,
      BodyNodeEnum bodyNode) const = 0;

  virtual void getManipulabilityMeasureGradientBodyToBody(
      Eigen::VectorXd& manipulabilityMeasureGradient,
      BranchEnum fromBranch,
      BodyNodeEnum fromBodyNode,
      BranchEnum toBranch,
      BodyNodeEnum toBodyNode) const = 0;

  /************************
   * Get Hessian methods *
   ************************/

  /*! Gets the 6xDOF sptial Hessian matrix,
   * which is the derivative of the spatial Jacobian from world to body
   * w.r.t. the given generalized coordinate with index qIndex (Quaternion).
   * Note that this method only changes the non-zero entries of the Hessian matrix.
   * The Hessian needs to have the correct size.
   * @param hessian   Hessian matrix
   * @param qIndex    index of generalized coordinate
   * @param branch    branch the body belongs to
   * @param node      node of the body
   * @param frame     coordinate frame the Hessian is expressed in
   * @returns true if the matrix has non-zero entries.
   */
  virtual bool getHessianSpatialWorldToBodyForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF translational Hessian matrix,
   * which is the derivative of the Jacobian from world to body
   * w.r.t. the given generalized coordinate with index qIndex (Quaternion).
   * Note that this method only changes the non-zero entries of the Hessian matrix.
   * The Hessian needs to have the correct size.
   * @param hessian   Hessian matrix
   * @param qIndex    index of generalized coordinate
   * @param branch    branch the body belongs to
   * @param node      node of the body
   * @param frame     coordinate frame the Hessian is expressed in
   * @returns true if the matrix has non-zero entries.
   *
   * @note This method is unit tested.
   */
  virtual bool getHessianTranslationWorldToBodyForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF translational Hessian matrix,
   * which is the derivative of the Jacobian from world to body
   * w.r.t. the given generalized coordinate with index qIndex (Quaternion).
   * Note that this method only changes the non-zero entries of the Hessian matrix.
   * This also returns the spatial Jacobian used to compute the Hessian matrix.
   * The Hessian needs to have the correct size.
   * @param hessian           Hessian matrix
   * @param qIndex            index of generalized coordinate
   * @param branch            branch the body belongs to
   * @param node              node of the body
   * @param frame             coordinate frame the Hessian is expressed in
   * @param spatialJacobian   the spatial Jacobian used to compute the Hessian
   * @returns                 true if the matrix has non-zero entries.
   *
   * @note This method is unit tested.
   */
  virtual bool getHessianTranslationWorldToBodyForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame,
      Eigen::MatrixXd& spatialJacobian) const = 0;

  /*! Gets the 3xDOF translational Hessian matrix,
   * which is the derivative of the Jacobian from world to the center of mass of
   * the robot w.r.t. the given generalized coordinate with index qIndex (Quaternion).
   * Note that this method only changes the non-zero entries of the Hessian matrix.
   * The Hessian needs to have the correct size.
   * @param hessian   Hessian matrix
   * @param qIndex    index of generalized coordinate
   * @param branch    branch the body belongs to
   * @param node      node of the body
   * @param frame     coordinate frame the Hessian is expressed in
   * @returns true if the matrix has non-zero entries.
   *
   * @note This method is unit tested.
   */
  virtual bool getHessianTranslationWorldToComForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      CoordinateFrameEnum frame) const = 0;

  /*! Gets the 3xDOF rotational Hessian matrix,
   * which is the derivative of the Jacobian from world to body
   * w.r.t. the given generalized coordinate with index qIndex (Quaternion).
   * Note that this method only changes the non-zero entries of the Hessian matrix.
   * The Hessian needs to have the correct size.
   * @param hessian   Hessian matrix
   * @param qIndex    index of generalized coordinate
   * @param branch    branch the body belongs to
   * @param node      node of the body
   * @param frame     coordinate frame the Hessian is expressed in
   * @returns true if the matrix has non-zero entries.
   *
   * @note This method is unit tested.
   */
  virtual bool getHessianRotationWorldToBodyForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;
  /************************/


  /************************
   * Get velocity methods *
   ************************/

  /*! @returns the (absolute) linear velocity of the body.
   *
   * @param body          enum representing the body in the kinematic chain.
   * @param frame         coordinate frame the velocity is expressed in
   */
  virtual Eigen::Vector3d getLinearVelocityWorldToBody(
      BodyEnum body,
      CoordinateFrameEnum frame) const = 0;

  /*! @returns the (absolute) linear velocity of the body.
   *
   * @param branch        enum representing a branch in the kinematic chain
   * @param bodyNode      enum representing a node in the branch
   * @param frame         coordinate frame the velocity is expressed in
   */
  virtual Eigen::Vector3d getLinearVelocityWorldToBody(
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const = 0;

  /*! @returns the linear velocity of a point on a body.
   *
   * @param bodyEnum      enum representing a body
   * @param point         a point on the body in local body coordinates
   * @param frame         coordinate frame the velocity is expressed in
   */
  virtual Eigen::Vector3d getLinearVelocityWorldToPointOnBody(
      BodyEnum body,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      CoordinateFrameEnum frame) const = 0;

  /*! @returns the linear velocity of a point on a body.
   *
   * @param branch        branche the body belongs to
   * @param node          noded of the body
   * @param point         a point on the body in local body coordinates
   * @param frame         coordinate frame the velocity is expressed in
   */
  virtual Eigen::Vector3d getLinearVelocityWorldToPointOnBody(
      BranchEnum branch,
      BodyNodeEnum node,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      CoordinateFrameEnum frame) const = 0;

  /*! @returns the relative linear velocity between a point of a body and the origin of another body
   *
   * @param fromBody      enum representing a body
   * @param toBody        enum representing a body
   * @param point         a point on the body (represented by toBody) in local body coordinates
   * @param frame         coordinate frame the velocity is expressed in
   */
  virtual Eigen::Vector3d getLinearVelocityBodyToPointOnBody(
      BodyEnum fromBody,
      BodyEnum toBody,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      CoordinateFrameEnum frame) const = 0;

  /*! @returns the relative linear velocity between a point of a body and the origin of another body
   *
   * @param fromBody      enum representing a body
   * @param toBody        enum representing a body
   * @param point         a point on the body (represented by toBody) in local body coordinates
   * @param frame         coordinate frame the velocity is expressed in
   */
  virtual Eigen::Vector3d getLinearVelocityWorldToBodyCom(
      BodyEnum body, CoordinateFrameEnum frame) const = 0;

  /*! @returns the linear velocity of the center of mass of the robot.
   * @param frame   coordinate frame the velocity is expressed in
   */
  virtual Eigen::Vector3d getLinearVelocityWorldToCom(CoordinateFrameEnum frame) const = 0;


  /*! @returns the (absolute) angular velocity of the body.
   *
   * @param body          enum representing the body in the kinematic chain.
   * @param frame         coordinate frame the velocity is expressed in
   */
  virtual Eigen::Vector3d getAngularVelocityWorldToBody(
      BodyEnum body,
      CoordinateFrameEnum frame) const = 0;

  /*! @returns the (absolute) angular velocity of the body.
   *
   * @param branch        enum representing a branch in the kinematic chain
   * @param bodyNode      enum representing a node in the branch
   * @param frame         coordinate frame the velocity is expressed in
   */
  virtual Eigen::Vector3d getAngularVelocityWorldToBody(
      BranchEnum branch,
      BodyNodeEnum bodyNode,
      CoordinateFrameEnum frame) const = 0;
  /************************/

  /************
   * Dynamics *
   ************/

  /*!
   * Get the absolute value of the gravity acceleration
   * @returns the absolute value of the gravity acceleration
   */
  virtual double getGravityAcceleration() const = 0;

  /*!
   * Get the gravity vector in world frame
   * @returns the gravity vector in world frame
   */
  virtual const Eigen::Vector3d& getGravityVectorInWorldFrame() const = 0;

  /*! Get the mass of a body.
   * @param body      the target body enum
   * @returns the mass of a body
   */
  virtual double getBodyMass(BodyEnum body) const = 0;

  /*! Get the mass of a body.
   * @param branch    the branch of the the target body enum
   * @param node      the node of the the target body enum
   * @returns the mass of a body
   */
  virtual double getBodyMass(BranchEnum branch, BodyNodeEnum node) const = 0;

  /*! Set the mass of a movable body.
   * This body must be movable; fixed children of the body have their mass properties set to zero.
   * @param body               the target body enum
   * @param mass               the mass of the body
   * @param updateBodyInertia  whether to compute the new movable body inertia properties
   * @returns true if successful
   */
  virtual bool setBodyMass(BodyEnum body, double mass, bool updateBodyInertia = true) const = 0;

  /*! Set the mass of a body.
   * This body can be movable or fixed; fixed children of this body are not affected
   * @param body               the target body enum
   * @param mass               the mass of the body
   * @param updateBodyInertia  whether to compute the new movable body inertia properties
   * @returns true if successful
   */
  virtual bool setIndividualBodyMass(BodyEnum body, double mass, bool updateBodyInertia = true) const = 0;

  /*! Get mass of the root body. This includes the mass of all the fixed bodies attached to it.
   * @returns the mass of the main body
   */
  virtual double getRootMass() const = 0;

  /*! Get the total mass of a branch
   * @param branch    the target branch
   * @returns the total mass of a branch
   */
  virtual double getLimbMass(BranchEnum branch) const = 0;

  /*! Get total mass of the robot.
   * @returns the mass of the robot
   */
  virtual double getTotalMass() const = 0;

  /*! Updates the inertia properties
   * @param body      the target body enum
   * @returns true if successful, false otherwise
   */
  virtual bool updateBodyInertia(BodyEnum body) const = 0;

  /*! Get the inertia tensor of a body, including the inertias of all the fixed bodies attached to it.
   * @param body      the target body enum
   * @returns a const reference to the intertia tensor of the body
   */
  virtual const Eigen::Matrix3d& getBodyInertiaMatrix(BodyEnum body) const = 0;

  /*! Set the inertia tensor of a body, including the inertias of all the fixed bodies attached to it.
   *  The body must be a movable body. Fixed bodies attached to it will have their mass properties set to zero.
   * @param body               the target body enum
   * @param inertiaMatrix      the inertia tensor of the body
   * @param updateBodyInertia  whether to compute the new movable body inertia properties
   * @returns         true if successful
   */
  virtual bool setBodyInertiaMatrix(BodyEnum body, const Eigen::Matrix3d& inertiaMatrix, bool updateBodyInertia = true) const = 0;

  /*! Set the inertia tensor of a body, without the fixed bodies attached to it.
   *  The body can be either movable or fixed. Fixed children are not affected.
   * @param body               the target body enum
   * @param inertiaMatrix      the inertia tensor of the body
   * @param updateBodyInertia  whether to compute the new movable body inertia properties
   * @returns         true if successful
   */
  virtual bool setIndividualBodyInertiaMatrix(BodyEnum body, const Eigen::Matrix3d& inertiaMatrix, bool updateBodyInertia = true) const = 0;

  /*! Set the inertia properties of a body, including the inertias of all the fixed bodies attached to it, and updates the mass properties.
   *  The body must be a movable body. Fixed bodies attached to it will have their mass properties set to zero.
   * @param body                     the target body enum
   * @param mass                     the mass of the body
   * @param centerOfMassInBodyFrame  the center of mass in the body frame
   * @param inertiaMatrix            the inertia tensor of the body
   * @param updateBodyInertia        whether to compute the new movable body inertia properties
   * @returns         true if successful
   */
  virtual bool setBodyInertiaProperties(BodyEnum body,
                                        const double mass,
                                        const Eigen::Vector3d& centerOfMassInBodyFrame,
                                        const Eigen::Matrix3d& inertiaAtCom,
                                        bool updateBodyInertia = true) const = 0;

  /*! Set the inertia properties of a body, without the fixed bodies attached to it.
   *  The body can be either movable or fixed. Fixed children are not affected.
   * @param body                     the target body enum
   * @param mass                     the mass of the body
   * @param centerOfMassInBodyFrame  the center of mass in the body frame
   * @param inertiaMatrix            the inertia tensor of the body
   * @param updateBodyInertia        whether to compute the new movable body inertia properties
   * @returns         true if successful
   */
  virtual bool setIndividualBodyInertiaProperties(BodyEnum body,
                                                  const double mass,
                                                  const Eigen::Vector3d& centerOfMassInBodyFrame,
                                                  const Eigen::Matrix3d& inertiaAtCom,
                                                  bool updateBodyInertia = true) const = 0;

  /*! Get the gravity terms of the equations of motion. Only the nonzero entries are computed.
   * @param gravity    the output of the function. This must be initialized to a zero Dof x 1 vector
   * @returns true
   */
  virtual bool getGravityTerms(Eigen::VectorXd& gravity) const = 0;

  /*! Compute the gravity terms of the equations of motion and return a copy of it.
   * @returns the D.O.F. x 1 vector containing the generalized forces due to gravity that are acting on each degree of freedom
   */
  virtual Eigen::VectorXd getGravityTerms() const = 0;

  /*! Get the coriolis, centrifugal and gravity terms of the equations of motion. Only the nonzero entries are computed.
   * @param nonlinearEffects    the output of the function. This must be initialized to a zero Dof x 1 vector
   * @returns true
   */
  virtual bool getNonlinearEffects(Eigen::VectorXd& nonlinearEffects) const = 0;

  /*! Compute mass matrix of the robot and return a copy of it.
   * @returns the D.O.F. x 1 vector containing the generalized forces due to the coriolis, centrifugal and gravity terms that are acting on each degree of freedom
   */
  virtual Eigen::VectorXd getNonlinearEffects() const = 0;

  /*! Get mass matrix of the robot. The method computes only the nonzero entries of the matrix.
   * @param massMatrix    the output of the function. This must be initialized to a zero Dof x Dof matrix
   * @returns true
   */
  virtual bool getMassInertiaMatrix(Eigen::MatrixXd& massMatrix) const = 0;

  /*! Compute mass matrix of the robot and return a copy of it.
   * @returns the D.O.F. x D.O.F. mass matrix of the robot
   */
  virtual Eigen::MatrixXd getMassInertiaMatrix() const = 0;

  /*! Compute the dynamic parameters related to the gravity terms.
   * For each link, four parameters are collected (mass m and center of mass expressed in
   * body fixed frame k_r_ks) as follows:
   * v = [m0 0_r_0s_x 0_r_0s_y 0_r_ks_z m1 1_r_1s_x 1_r_1s_y 1_r_1s_z ... ]
   * @returns true if succeeded
   */
  virtual bool getGravityDynamicParameters(Eigen::VectorXd& parameters) const = 0;

  /*! Computes the gravity terms given joint positions and link parameters.
   * The link parameters must be of the form that is returned by getGravityDynamicParameters.
   *
   * This method only changes the model temporarily.
   * @returns true if succeeded
   */
  virtual bool getJointGravityTermsFromJointConfigurationAndLinkParameters(
      Eigen::VectorXd& gravityTerms, const Eigen::VectorXd& jointPositions,
      const Eigen::VectorXd& dynamicParams) = 0;

  /************/

  /*! Get a body in the kinematic structure.
 * @param body  the enum of the the target body enum
 * @returns a const reference to an object that holds the kinematic and dynamics information of a given body.
 */
  virtual const romo::RigidBody<ConcreteDescription_>& getBody(BodyEnum body) const = 0;

  /*! Get a body in the kinematic structure.
   * @param branch    the branch of the the target body enum
   * @param node      the node of the the target body enum
   * @returns a const reference to an object that holds the kinematic and dynamics information of a given body.
   */
  virtual const romo::RigidBody<ConcreteDescription_>& getBody(BranchEnum branch, BodyNodeEnum node) const = 0;

  virtual double getTimeStep() const = 0;

  /**
   * @brief      Returns the limb positions from the endeffector position with
   *             iterative inverse kinematics
   *
   * @param[out] limbJointPositions                          The resulting limb
   *                                                         joint positions
   * @param[in]  desiredPositionBaseToTargetBodyInBaseFrame  The desired
   *                                                         position base to
   *                                                         the target body in
   *                                                         base frame
   * @param[in]  contactEnum                                 The contact enum
   * @param[in]  toleranceForFailCheck                       The tolerance for fail check
   * @param[in]  toleranceForIteration                       The tolerance for iteration
   * @param[in]  maxIterations                               The maximum iterations
   * @param[in]  verbose                                     Prints debug output
   *                                                         if true
   *
   * @return     true if the error of the solution does not exceed to
   *             toleranceForFailCheck
   */
  virtual bool getLimbJointPositionsFromContactEnumIteratively (
    Eigen::VectorXd& limbJointPositions,
    const Eigen::Vector3d& desiredPositionBaseToTargetPointInBaseFrame,
    const ContactEnum contactEnum,
    const double toleranceForFailCheck = 1e-2,
    const double toleranceForIteration = 1e-12,
    const unsigned int maxIterations = 10,
    const bool verbose = false);

    /**
     * @brief      Returns the limb positions from the endeffector position with
     *             iterative inverse kinematics
     *
     * @param[out] limbJointPositions                           The resulting
     *                                                          limb joint
     *                                                          positions
     * @param[in]  desiredPositionBaseToTargetPointInBaseFrame  The desired
     *                                                          position base to
     *                                                          the target body
     *                                                          in base frame
     * @param[in]  limbEnum                                     The limb enum
     * @param[in]  toleranceForFailCheck                        The tolerance for fail check
     * @param[in]  toleranceForIteration                        The tolerance for iteration
     * @param[in]  maxIterations                                The maximum iterations
     * @param[in]  verbose                                      Prints debug
     *                                                          output if true
     *
     * @return     true if the error of the solution does not exceed to
     *             toleranceForFailCheck
     */
  virtual bool getLimbJointPositionsFromLimbEnumIteratively (
    Eigen::VectorXd& limbJointPositions,
    const Eigen::Vector3d& desiredPositionBaseToTargetPointInBaseFrame,
    const LimbEnum limbEnum,
    const double toleranceForFailCheck = 1e-2,
    const double toleranceForIteration = 1e-12,
    const unsigned int maxIterations = 10,
    const bool verbose = false);

  /**
   * @brief      Returns the limb positions from the contact position with
   *             iterative inverse kinematics
   *
   * @param[out] limbJointPositions                          The resulting limb
   *                                                         joint positions
   * @param[in]  desiredPositionBaseToTargetBodyInBaseFrame  The desired
   *                                                         position base to
   *                                                         the target body in
   *                                                         base frame   *
   * @param[in]  bodyEnum                                    The limb enum
   * @param[in]  positionBodyToTargetPointInBodyFrame        The position base
   *                                                         to endeffector in
   *                                                         base frame
   * @param[in]  toleranceForFailCheck                       The tolerance for fail check
   * @param[in]  toleranceForIteration                       The tolerance for iteration
   * @param[in]  maxIterations                               The maximum iterations
   * @param[in]  verbose                                     Prints debug output
   *                                                         if true
   * @param[in]  updateJoints                                The update joints
   *
   * @return     true if the error of the solution does not exceed to
   *             toleranceForFailCheck
   */
  virtual bool getLimbJointPositionsFromPositionBaseToTargetPointInBaseFrameIteratively(
    Eigen::VectorXd& limbJointPositions,
    const Eigen::Vector3d& desiredPositionBaseToTargetPointInBaseFrame,
    const BodyEnum bodyEnum,
    const Eigen::Vector3d& positionBodyToTargetPointInBodyFrame,
    const double toleranceForFailCheck = 1e-2,
    const double toleranceForIteration = 1e-12,
    const unsigned int maxIterations = 10,
    const bool verbose = false,
    std::function<void(JointPositions&, const Eigen::VectorXd&, const LimbEnum)> updateJoints =
      [](JointPositions& positions, const Eigen::VectorXd& jointUpdate, const LimbEnum limbEnum)
        {
          const auto limbStartId        = RD::getLimbStartIndexInJ(limbEnum);
          const auto numDofInvKin       = RD::getNumDofLimb(limbEnum);
          positions.toImplementation().template segment(limbStartId, numDofInvKin) += jointUpdate;
        }
    );

  /**
   * @brief      Returns the limb joint positions from an endeffector pose using
   *             iterative inverse kinematics
   *
   * @param[out] limbJointPositions                      The resulting limb
   *                                                     joint positions
   * @param[in]  desiredPoseBaseToTargetBodyInBaseFrame  The pose base to
   *                                                     endeffector in base
   *                                                     frame
   * @param[in]  limbEnum                                The limb enum
   * @param[in]  doReconfiguration                       Apply nullspace
   *                                                     reconfiguration if true
   * @param[in]  q_des                                   The desired joint
   *                                                     configuration for
   *                                                     reconfiguration in
   *                                                     singularities
   * @param[in]  updateGain                              The position update
   *                                                     gain
   * @param[in]  toleranceForFailCheck                   The tolerance for fail check
   * @param[in]  toleranceForIteration                   The tolerance for iteration
   * @param[in]  maxIterations                           The maximum iterations
   * @param[in]  verbose                                 Prints debug output if
   *                                                     true
   *
   * @return     true if the error of the solution does not exceed to
   *             toleranceForFailCheck
   */
  virtual bool getLimbJointPositionsFromPoseEndEffectorToBaseIteratively (
    Eigen::VectorXd& limbJointPositions,
    const Pose& desiredPoseEndEffectorToBase,
    const LimbEnum limbEnum,
    const bool doReconfiguration = false,
    const Eigen::VectorXd* desiredJoints = nullptr,
    const double updateGain = 1.0,
    const double toleranceForFailCheck = 1.0e-2,
    const double toleranceForIteration = 1.0e-3,
    const unsigned int maxIterations = 100,
    const bool verbose = false);
};

}  // namespace romo

#include <romo/RobotModel.tpp>
