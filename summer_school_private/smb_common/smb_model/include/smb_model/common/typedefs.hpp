/*
 * typedefs.hpp
 *
 *  Created on: Aug 1, 2018
 *      Author: Koen Kraemer
 */

#pragma once

// romo
#include <romo/common/phys_typedefs.hpp>
#include <romo/common/robot_state_typedefs.hpp>

// description
#include <smb_description/SmbDescription.hpp>

// kindr
#include <kindr/Core>

// eigen
#include <Eigen/Core>

namespace smb_model {

/* Robot Description */
using ConcreteDescription = romo::ConcreteDescription<smb_description::SmbDefinitions, smb_description::SmbTopology>;
using RD = romo::RobotDescription<ConcreteDescription>;

// Import types from the robot model namespace.
using romo::Pose;
using romo::Twist;
using romo::RotationQuaternion;
using romo::RotationQuaternionDiff;
using romo::AngleAxis;
using romo::RotationMatrix;
using romo::EulerAnglesZyx;
using romo::RotationVector;
using romo::EulerAnglesXyz;
using romo::EulerAnglesXyzDiff;
using romo::Position;
using romo::LinearVelocity;
using romo::LocalAngularVelocity;
using romo::EulerAnglesZyxDiff;
using romo::LinearAcceleration;
using romo::AngularAcceleration;
using romo::Force;
using romo::Torque;
using romo::Vector;

/* Joint-Sized Vectors */
template <typename Scalar_>
using JointVector = Eigen::Matrix<Scalar_, RD::getJointsDimension(), 1>;
using JointVectorD = JointVector<double>;
using JointVectorB = JointVector<bool>;
using JointVectorI = JointVector<int>;

/* Joints */
using JointPositions = romo::internal::JointPositions<RD::getJointsDimension()>;
using JointVelocities = romo::internal::JointVelocities<RD::getJointsDimension()>;
using JointAccelerations = romo::internal::JointAccelerations<RD::getJointsDimension()>;
using JointTorques = romo::internal::JointTorques<RD::getJointsDimension()>;

using JointPositionsLimb = romo::internal::JointPositions<RD::getNumDofLimb()>;
using JointVelocitiesLimb = romo::internal::JointVelocities<RD::getNumDofLimb()>;
using JointAccelerationsLimb = romo::internal::JointAccelerations<RD::getNumDofLimb()>;
using JointTorquesLimb = romo::internal::JointTorques<RD::getNumDofLimb()>;

/* Generalized Coordinates */
using GeneralizedCoordinates = romo::internal::GeneralizedCoordinates<RD::getGeneralizedCoordinatesDimension()>;
using GeneralizedVelocities = romo::internal::GeneralizedVelocities<RD::getGeneralizedVelocitiesDimension()>;
using GeneralizedAccelerations = romo::internal::GeneralizedAccelerations<RD::getGeneralizedVelocitiesDimension()>;

/* Dynamics */
using MassMatrix = Eigen::Matrix<double, RD::getGeneralizedVelocitiesDimension(), RD::getGeneralizedVelocitiesDimension()>;
using GravityTorqueVector = kindr::Torque<double, RD::getGeneralizedVelocitiesDimension()>;
using NonlinearEffectsVector = GravityTorqueVector;
using SelectionMatrix = Eigen::Matrix<double, RD::getJointsDimension(), RD::getGeneralizedVelocitiesDimension()> ;

/* Jacobians */
using Jacobian = Eigen::Matrix<double, RD::getNumTranslationalDof(), RD::getGeneralizedVelocitiesDimension()>;
using TranslationalJacobian = Eigen::Matrix<double, RD::getNumTranslationalDof(), RD::getGeneralizedVelocitiesDimension()>;
using RotationalJacobian = Eigen::Matrix<double, RD::getNumRotationalDof(), RD::getGeneralizedVelocitiesDimension()>;
using SpatialJacobian = Eigen::Matrix<double, (RD::getNumTranslationalDof() + RD::getNumRotationalDof()) , RD::getGeneralizedVelocitiesDimension()>;

} // namespace smb_model
