/*
 * Contact.hpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// romo
#include "romo/common/phys_typedefs.hpp"
#include "romo/common/RobotDescription.hpp"

namespace romo {

template<typename ConcreteDescription_>
class Contact {

 protected:
  using RD = RobotDescription<ConcreteDescription_>;

 public:
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;
  using ContactState = typename RD::ContactStateEnum;

  Contact() = default;
  virtual ~Contact() = default;

  virtual Force getForce(CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) const = 0;
  virtual void setForce(const Force& force, CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) = 0;

  virtual Torque getTorque(CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) const = 0;
  virtual void setTorque(const Torque& torque, CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) = 0;

  virtual Vector getNormal(CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) const = 0;
  virtual void setNormal(const Vector& normal, CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) = 0;

  virtual const ContactState& getState() const = 0;
  virtual void setState(ContactState state) = 0;

  virtual const Position& getPositionMovableParentToContactInMovableParentFrame() const = 0;
  virtual void setPositionMovableParentToContactInMovableParentFrame(Position position) = 0;

  virtual const RotationQuaternion& getOrientationMovableParentToContact() const = 0;
  virtual void setOrientationMovableParentToContact(RotationQuaternion orientation) = 0;

  virtual Position getPositionWorldToContact(CoordinateFrameEnum frame) const = 0;
  virtual RotationQuaternion getOrientationWorldToContact() const = 0;

  virtual bool getJacobianSpatialWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const = 0;
  virtual bool getJacobianSpatialTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const = 0;
  virtual bool getJacobianSpatialTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobianDerivative,
                                                               const Eigen::MatrixXd& spatialJacobian,
                                                               CoordinateFrameEnum frame) const = 0;

  virtual bool getJacobianTranslationWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const = 0;
  virtual bool getJacobianTranslationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const = 0;
  virtual bool getJacobianTranslationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian,
                                                                   const Eigen::MatrixXd& spatialJacobian,
                                                                   CoordinateFrameEnum frame) const = 0;

  virtual bool getJacobianRotationWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const = 0;
  virtual bool getJacobianRotationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const = 0;
  virtual bool getJacobianRotationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian,
                                                                const Eigen::MatrixXd& spatialJacobianInWorldFrame,
                                                                CoordinateFrameEnum frame) const = 0;

  virtual LinearVelocity getLinearVelocityWorldToContact( CoordinateFrameEnum frame) const = 0;
  virtual LocalAngularVelocity getAngularVelocityWorldToContact( CoordinateFrameEnum frame) const = 0;

};

} /* namespace romo */
