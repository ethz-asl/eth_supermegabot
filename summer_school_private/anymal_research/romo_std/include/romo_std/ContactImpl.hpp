/*
 * ContactImpl.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// romo
#include "romo/common/Contact.hpp"
#include "romo/RobotModel.hpp"

// Eigen
#include <Eigen/Core>

// STL
#include <memory>

namespace romo_std {



template<typename ConcreteDescription_, typename RobotState_>
class ContactImpl : public romo::Contact<ConcreteDescription_>
{
  using Base = romo::Contact<ConcreteDescription_>;
  using RobotModelPtr = romo::RobotModel<ConcreteDescription_, RobotState_>*;

 protected:
  using RD              = typename Base::RD;
  using ContactEnum     = typename RD::ContactEnum;
  using BodyNodeEnum    = typename RD::BodyNodeEnum;
  using BranchEnum      = typename RD::BranchEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;
  using ContactState    = typename RD::ContactStateEnum;

 public:
  ContactImpl() = default;
  ContactImpl(RobotModelPtr model, ContactEnum contactEnum);
  virtual ~ContactImpl() = default;

  romo::Force getForce(CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) const override;
  void setForce(const romo::Force& force, CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) override;

  romo::Vector getNormal(CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) const override;
  void setNormal(const romo::Vector& normal, CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) override;

  romo::Torque getTorque(CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) const override;
  void setTorque(const romo::Torque& torque, CoordinateFrameEnum frame = CoordinateFrameEnum::WORLD) override;

  const ContactState& getState() const override;
  void setState(ContactState state) override;

  const romo::Position& getPositionMovableParentToContactInMovableParentFrame() const override;
  void setPositionMovableParentToContactInMovableParentFrame(romo::Position position) override;

  const romo::RotationQuaternion& getOrientationMovableParentToContact() const override;
  void setOrientationMovableParentToContact(romo::RotationQuaternion orientation) override;

  romo::Position getPositionWorldToContact(CoordinateFrameEnum frame) const override;
  romo::RotationQuaternion getOrientationWorldToContact() const override;

  bool getJacobianSpatialWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const override;
  bool getJacobianSpatialTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const override;
  bool getJacobianSpatialTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobianDerivative,
                                                               const Eigen::MatrixXd& spatialJacobian,
                                                               CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const override;
  bool getJacobianTranslationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const override;
  bool getJacobianTranslationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian,
                                                                   const Eigen::MatrixXd& spatialJacobian,
                                                                   CoordinateFrameEnum frame) const override;

  bool getJacobianRotationWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const override;
  bool getJacobianRotationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian, CoordinateFrameEnum frame) const override;
  bool getJacobianRotationTimeDerivativeWorldToContact( Eigen::MatrixXd& jacobian,
                                                                const Eigen::MatrixXd& spatialJacobianInWorldFrame,
                                                                CoordinateFrameEnum frame) const override;

  romo::LinearVelocity getLinearVelocityWorldToContact( CoordinateFrameEnum frame) const override;
  romo::LocalAngularVelocity getAngularVelocityWorldToContact( CoordinateFrameEnum frame) const override;

 protected:
  RobotModelPtr model_;

  romo::Force forceInWorldFrame_;
  romo::Torque torqueInWorldFrame_;
  romo::Vector normalInWorldFrame_;
  romo::Position positionMovableParentToContactInMovableParentFrame_;
  romo::RotationQuaternion orientationMovableParentToContact_;

  ContactState state_;
  ContactEnum  contactEnum_;
  BranchEnum  branchEnum_;
  BodyNodeEnum  bodyNodeEnum_;
};

} /* namespace romo */

#include "ContactImpl.tpp"
