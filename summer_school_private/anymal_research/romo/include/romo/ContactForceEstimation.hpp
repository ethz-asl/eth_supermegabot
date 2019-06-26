/*
 * ContactForceEstimation.hpp
 *
 *  Created on: Nov, 2017
 *      Author: Gabriel Hottiger, Dario Bellicoso
 */

#pragma once

#include <romo/RobotModel.hpp>

#include <Eigen/Core>

#include <message_logger/message_logger.hpp>
#include <robot_utils/filters/FirstOrderFilter.hpp>
#include <kindr/math/LinearAlgebra.hpp>

namespace romo {

template<typename ConcreteDescription_, typename RobotState_>
class ContactForceEstimation {

  using Model = RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename Model::RD;
  using NonlinearEffects = typename RD::NonlinearEffects;
  using MassMatrix = typename RD::MassMatrix;
  using JointVector = typename RD::JointVector;

  // constexpr auto dim_ = (T == ContactForceEstimationType::FULL) ? RD::getNumDof() : RD::getJointsDimension();
  static constexpr auto dim_ = RD::getJointsDimension();
  static constexpr auto contactForcesDimension_ = RD::getNumTranslationalDof() * RD::getNumLimbs();
  using StackedContactForces = Eigen::Matrix<double, contactForcesDimension_, 1>;
  using StackedSupportJacobian = Eigen::Matrix<double, contactForcesDimension_, dim_>;
  using StackedSupportJacobianTransposed = Eigen::Matrix<double, dim_, contactForcesDimension_>;

 public:
  explicit ContactForceEstimation(const Model& model, double dt = 0.01)
    : model_(model)
  {
    jointAccelerationFilters_.setFilterParameters(dt, 1.0e-2, 1.0, JointVector::Zero());
  }

  bool estimate() {
    // Calculate unfiltered joint accelerations
    const JointVector& jointVelocities = model_.getState().getJointVelocities().toImplementation();
    const JointVector jointAccelerations = (jointVelocities - jointVelocitiesOld_) / model_.getTimeStep();
    jointVelocitiesOld_ = jointVelocities;
    return estimate(jointAccelerationFilters_.advance(jointAccelerations));
  }

  bool estimate(const JointVector& jointAccelerations) {
    // Calculate dynamics
    NonlinearEffects nonlinearEffects = model_.getNonlinearEffects();
    MassMatrix massMatrix = model_.getMassInertiaMatrix();

    // Calculated stacked support jacobian
    StackedSupportJacobian supportJacobianInWorldFrame = StackedSupportJacobian::Zero();
    unsigned int startRow = 0;
    for(auto & contact : model_.getContactContainer() ) {
      // Todo remove temporary once romo is fixed size
      Eigen::MatrixXd temp = RD::JacobianTranslation::Zero();
      contact->getJacobianTranslationWorldToContact(temp, RD::CoordinateFrameEnum::WORLD);
      supportJacobianInWorldFrame.template middleRows<RD::getNumTranslationalDof()>(startRow) = temp.rightCols<dim_>();
      startRow += RD::getNumTranslationalDof();
    }

    // Calculate pseudoInverse of transpose
    const StackedSupportJacobianTransposed supportJacobianTransposeInWorldFrame =
      supportJacobianInWorldFrame.transpose();
    StackedSupportJacobian supportJacobianTransposePsuedoInverseInWorldFrame =
      robot_utils::pseudoInverseAdaptiveDls(supportJacobianTransposeInWorldFrame);

    // estimate the contact forces
    StackedContactForces estimatedContactForces = -supportJacobianTransposePsuedoInverseInWorldFrame
      * (model_.getState().getJointTorques().toImplementation()
        - nonlinearEffects.template tail<dim_>()
        - massMatrix.template bottomRightCorner<dim_, dim_>() * jointAccelerations);

    // set contact forces
    startRow = 0;
    for(auto & contact : model_.getContactContainer() ) {
      contact->setForce(Force(estimatedContactForces.template segment<3>(startRow)));
      startRow += RD::getNumTranslationalDof();
    }
    return true;
  }

 private:
  const Model & model_;
  JointVector jointVelocitiesOld_;
  robot_utils::FirstOrderFilter<JointVector> jointAccelerationFilters_;
};

} /* namespace romo */
