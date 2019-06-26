/*
 * kinematics.cpp
 *
 *  Created on: February 01, 2018
 *      Author: Yvain de Viragh
 */

#include "robot_utils/kinematics/kinematics.hpp"
// #include <iostream>

namespace robot_utils {

/*
 * Note: This implementation exploits that constant frame-fixed angular velocity implies constant reference angular velocity
 * and thus a fixed axis of rotation with a time-varying angle. The position after timeHorizonInSeconds can therefore be obtained by
 * integrating over time Rodrigue's formula applied on the linear velocity. We only need to be careful for
 * rotation angles close to zero, as these require taking the limit of the analytic solution.
 *
 */
kindr::HomTransformQuatD getPoseEndToReferenceFromTwistReferenceToStartInStartFrame(const kindr::HomTransformQuatD& poseStartToReference,
                                                                                    const kindr::TwistLocalD& twistReferenceToStartInStartFrame,
                                                                                    double timeHorizonInSeconds) {
  const kindr::RotationMatrixPD& rotationStartToReference = kindr::RotationMatrixPD(poseStartToReference.getRotation());
  const kindr::Velocity3D& linearVelocityReferenceToStartInStartFrame = twistReferenceToStartInStartFrame.getTranslationalVelocity();
  const kindr::AngularVelocity3D& angularVelocityReferenceToStartInStartFrame = twistReferenceToStartInStartFrame.getRotationalVelocity();

  double angularSpeed = angularVelocityReferenceToStartInStartFrame.norm();
  double angle = timeHorizonInSeconds * angularSpeed;

  if (areNear(angle,0.0)) { // Check whether we need to take the limit of the analytical solution.

    kindr::Position3D positionReferenceToEndInReferenceFrame = kindr::Position3D(
        poseStartToReference.getPosition().vector() + timeHorizonInSeconds * rotationStartToReference.matrix() * linearVelocityReferenceToStartInStartFrame.vector());
    kindr::RotationMatrixPD rotationEndToReference = rotationStartToReference;

    return kindr::HomTransformQuatD(positionReferenceToEndInReferenceFrame, kindr::RotationQuaternionPD(rotationEndToReference));

  } else {

    kindr::VectorTypeless3D rotationAxis(angularVelocityReferenceToStartInStartFrame.vector() / angularSpeed);
    auto rotationAxisSkew = kindr::getSkewMatrixFromVector(rotationAxis.vector());

    // Time integral of Rodrigue's formula
    kindr::Position3D positionReferenceToEndInReferenceFrame = kindr::Position3D(
        poseStartToReference.getPosition().vector() + rotationStartToReference.matrix() * (Eigen::Matrix3d::Identity()*timeHorizonInSeconds
        + 1/angularSpeed * (1.0 - std::cos(angle)) * rotationAxisSkew
        + (timeHorizonInSeconds - 1.0/angularSpeed * std::sin(angle)) * rotationAxisSkew * rotationAxisSkew) * linearVelocityReferenceToStartInStartFrame.vector());

    kindr::RotationMatrixPD rotationEndToReference = rotationStartToReference * kindr::RotationMatrixPD(kindr::AngleAxisD(angle,rotationAxis.vector()));

    return kindr::HomTransformQuatD(positionReferenceToEndInReferenceFrame, kindr::RotationQuaternionPD(rotationEndToReference));
  }
}

/*
 * Note: The following approximation is used:
 *
 * finalPose = initialPose + initialOrientation * eulerVectorToRotationMatrix(t*angularVelocityInMovingFrame) * linearVelocityInMovingFrame * t;
 *
 */
kindr::HomTransformQuatD getApproximatedPoseEndToReferenceFromTwistReferenceToStartInStartFrame(const kindr::HomTransformQuatD& poseStartToReference,
                                                                                                const kindr::TwistLocalD& twistReferenceToStartInStartFrame,
                                                                                                double timeHorizonInSeconds) {
  const kindr::RotationMatrixPD& rotationStartToReference = kindr::RotationMatrixPD(poseStartToReference.getRotation());
  const kindr::Velocity3D& linearVelocityReferenceToStartInStartFrame = twistReferenceToStartInStartFrame.getTranslationalVelocity();
  const kindr::AngularVelocity3D& angularVelocityReferenceToStartInStartFrame = twistReferenceToStartInStartFrame.getRotationalVelocity();

  double angularSpeed = angularVelocityReferenceToStartInStartFrame.norm();
  double angle = timeHorizonInSeconds * angularSpeed;

  if (robot_utils::areNear(angularSpeed,0.0)) { // Avoid a division by zero

    kindr::Position3D positionReferenceToEndInReferenceFrame = kindr::Position3D(
        poseStartToReference.getPosition().vector() + timeHorizonInSeconds * rotationStartToReference.matrix() * linearVelocityReferenceToStartInStartFrame.vector());
    kindr::RotationMatrixPD rotationEndToReference = rotationStartToReference;

    return kindr::HomTransformQuatD(positionReferenceToEndInReferenceFrame, kindr::RotationQuaternionPD(rotationEndToReference));

  } else {

    kindr::VectorTypeless3D rotationAxis(angularVelocityReferenceToStartInStartFrame.vector() / angularSpeed);

    kindr::RotationMatrixPD rotationEndToReference = rotationStartToReference * kindr::RotationMatrixPD(kindr::AngleAxisD(angle,rotationAxis.vector()));

    kindr::Position3D positionReferenceToEndInReferenceFrame = kindr::Position3D(
        poseStartToReference.getPosition().vector() + timeHorizonInSeconds * rotationEndToReference.matrix() * linearVelocityReferenceToStartInStartFrame.vector());

    return kindr::HomTransformQuatD(positionReferenceToEndInReferenceFrame, kindr::RotationQuaternionPD(rotationEndToReference));
  }
}

} /* namespace robot_utils */
