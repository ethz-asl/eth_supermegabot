/*
 * kinematicsTest.hpp
 *
 *  Created on: February 03, 2018
 *      Author: Yvain de Viragh
 */

#include <gtest/gtest.h>
#include <kindr/common/gtest_eigen.hpp>
#include <kindr/Core>
#include "robot_utils/kinematics/kinematics.hpp"

/*
 * Very small rotations: This test checks whether the results for very  small rotations
 * - i.e. angular velocity or time are zero or very close to zero - are correct, since in
 * this case, the limit of the analytic solution needs to be taken.
 */
TEST(kinematicsTest, getPoseEndToReferenceFromTwistReferenceToStartInStartFrame_smallRotations)
{
  using namespace robot_utils;

  double timeHorizon;
  kindr::LocalAngularVelocityD angularVelocityReferenceToStartInStartFrame;
  std::string msg;

  // Arbitrary, nonzero choice for initial position, initial orientation, and linear velocity
  kindr::Position3D positionReferenceToStartInReferenceFrame(1.0,3.5,-0.5);
  kindr::RotationQuaternionPD rotationStartToReference = kindr::RotationQuaternionPD(kindr::EulerAnglesZyxPD(M_PI/4,-M_PI/3,M_PI/6));
  kindr::HomTransformQuatD poseStartToReference(positionReferenceToStartInReferenceFrame, rotationStartToReference);
  kindr::Velocity3D linearVelocityReferenceToStartInStartFrame(1.5,-0.4,0.8);

  for (unsigned int testIt = 0u; testIt < 4; testIt++) {
    switch (testIt) {
      case 0 :
        msg = "Zero angular velocity and large nonzero time";
        timeHorizon = 1.4e7;
        angularVelocityReferenceToStartInStartFrame << 0.0, 0.0, 0.0;
        break;
      case 1 :
        msg = "Large angular velocity and zero time";
        timeHorizon = 0.0;
        angularVelocityReferenceToStartInStartFrame << -4.5e7, 1.0e7, 8.7e7;
        break;
      case 2 :
        msg = "Very small angular velocity and nonzero time";
        timeHorizon = 2.5;
        angularVelocityReferenceToStartInStartFrame << 1.0e-10, -4.5e-10, 0;
        break;
      case 3 :
        msg = "Nonzero angular velocity and very small time";
        timeHorizon = 1e-10;
        angularVelocityReferenceToStartInStartFrame << 1.6, 7, -0.4;
    }
    kindr::Position3D realPositionReferenceToEndInReferenceFrame = positionReferenceToStartInReferenceFrame
        + rotationStartToReference.rotate(kindr::Position3D(timeHorizon * linearVelocityReferenceToStartInStartFrame));
    kindr::HomTransformQuatD realPoseEndToReference(realPositionReferenceToEndInReferenceFrame, rotationStartToReference);

    kindr::TwistLocalD twistReferenceToStartInStartFrame(linearVelocityReferenceToStartInStartFrame, angularVelocityReferenceToStartInStartFrame);
    kindr::HomTransformQuatD predictedPoseEndToReference = getPoseEndToReferenceFromTwistReferenceToStartInStartFrame(
        poseStartToReference,twistReferenceToStartInStartFrame,timeHorizon);

    KINDR_ASSERT_DOUBLE_MX_EQ(realPoseEndToReference.getTransformationMatrix(), predictedPoseEndToReference.getTransformationMatrix(), 1.0e-10, msg);
  }
};

/*
 * This test checks whether the function correctly computes the radial periodicity of the
 * helical motion: For rotation angles that are multiple of 2pi, the final position lies on
 * the line that has direction given by the angular velocity and goes through the initial position.
 */
TEST(kinematicsTest, getApproximatedPoseEndToReferenceFromTwistReferenceToStartInStartFrame_radialPeriodicity)
{
  using namespace robot_utils;

  // Choose the time and angular velocity such that the resulting rotation is a multiple of 2pi
  double timeHorizon = 2*M_PI;
  kindr::LocalAngularVelocityD angularVelocityReferenceToStartInStartFrame(1.0, -2.0, 2.0);

  // Arbitrary, nonzero choice for initial position, initial orientation, and linear velocity
  kindr::Position3D positionReferenceToStartInReferenceFrame(1.0, 3.5, -0.5);
  kindr::RotationQuaternionPD rotationStartToReference = kindr::RotationQuaternionPD(kindr::EulerAnglesZyxPD(M_PI/4,-M_PI/3,M_PI/6));
  kindr::HomTransformQuatD poseStartToReference(positionReferenceToStartInReferenceFrame, rotationStartToReference);
  kindr::Velocity3D linearVelocityReferenceToStartInStartFrame(1.5, -0.4, 0.8);

  // Final pose
  kindr::VectorTypeless3D globalRotationAxis((rotationStartToReference.rotate(angularVelocityReferenceToStartInStartFrame)).normalized());
  kindr::Position3D realPositionReferenceToEndInReferenceFrame = positionReferenceToStartInReferenceFrame
      + timeHorizon * kindr::Position3D(rotationStartToReference.rotate(linearVelocityReferenceToStartInStartFrame)).
      projectOn(kindr::Position3D(globalRotationAxis));
  kindr::HomTransformQuatD realPoseEndToReference(realPositionReferenceToEndInReferenceFrame, rotationStartToReference);

  kindr::TwistLocalD twistReferenceToStartInStartFrame(linearVelocityReferenceToStartInStartFrame, angularVelocityReferenceToStartInStartFrame);
  kindr::HomTransformQuatD predictedPoseEndToReference = getPoseEndToReferenceFromTwistReferenceToStartInStartFrame(
      poseStartToReference,twistReferenceToStartInStartFrame,timeHorizon);

  std::string msg{"Periodicity of helical motion"};
  KINDR_ASSERT_DOUBLE_MX_EQ(realPoseEndToReference.getTransformationMatrix(), predictedPoseEndToReference.getTransformationMatrix(), 1.0e-10, msg);

};

/*
 * This test checks the general correctness of the analytic solution. We look at the special
 * case of a rotation of pi/2 about the reference z-axis and where the linear velocity is aligned
 * with the reference x-axis, since this is straightforward to compute by hand without using the
 * formula the function does.
 */
TEST(kinematicsTest, getApproximatedPoseEndToReferenceFromTwistReferenceToStartInStartFrame_general)
{
  using namespace robot_utils;

  // Arbitrary choice for initial position
  kindr::Position3D positionReferenceToStartInReferenceFrame(1.0, 3.5, -0.5);

  // Arbitrary initial orientation
  kindr::RotationQuaternionPD rotationStartToReference(kindr::EulerAnglesZyxPD(M_PI/5, -M_PI/6, -5*M_PI/4));

  // Linear velocity aligned with x-axis of reference frame and arbitrary magnitude
  kindr::Velocity3D linearVelocityReferenceToStartInStartFrame(3.4 * kindr::RotationMatrixPD(rotationStartToReference).matrix().row(0).transpose());

  // Angular velocity aligned with z-axis of reference frame and arbitrary magnitude
  kindr::LocalAngularVelocityD angularVelocityReferenceToStartInStartFrame(0.8 * kindr::RotationMatrixPD(rotationStartToReference).matrix().row(2).transpose());

  // Time for a rotation angle of pi/2
  double timeHorizon = (M_PI/2)/angularVelocityReferenceToStartInStartFrame.norm();

  kindr::HomTransformQuatD poseStartToReference(positionReferenceToStartInReferenceFrame, rotationStartToReference);

  // Final pose
  kindr::Position3D realPositionReferenceToEndInReferenceFrame = positionReferenceToStartInReferenceFrame +
      linearVelocityReferenceToStartInStartFrame.norm() * timeHorizon/(M_PI/2) *  kindr::Position3D(1.0, 1.0, 0.0); // This term scales with time divided by the rotation angle
  kindr::RotationQuaternionPD realRotationEndToReference(kindr::EulerAnglesZyxPD(M_PI/5 + M_PI/2, -M_PI/6, -5*M_PI/4));
  kindr::HomTransformQuatD realPoseEndToReference(realPositionReferenceToEndInReferenceFrame, realRotationEndToReference);

  kindr::TwistLocalD twistReferenceToStartInStartFrame(linearVelocityReferenceToStartInStartFrame, angularVelocityReferenceToStartInStartFrame);
  kindr::HomTransformQuatD predictedPoseEndToReference = getPoseEndToReferenceFromTwistReferenceToStartInStartFrame(
      poseStartToReference,twistReferenceToStartInStartFrame,timeHorizon);

  std::string msg{"Correctness of analytic solution"};
  KINDR_ASSERT_DOUBLE_MX_EQ(realPoseEndToReference.getTransformationMatrix(), predictedPoseEndToReference.getTransformationMatrix(), 1.0e-10, msg);

};

/*
 * This test checks whether the approximative solution returns the expected result. We look at
 * the special case of a rotation of pi about the reference z-axis and where the linear velocity
 * is aligned with the reference x-axis, since this is straightforward to compute by hand without
 * using the formula the function does.
 */
TEST(kinematicsTest, getApproximatedPoseEndToReferenceFromTwistReferenceToStartInStartFrame)
{
  using namespace robot_utils;

  // Arbitrary choice for initial position and orientation
  kindr::Position3D positionReferenceToStartInReferenceFrame(1.0, 3.5, -0.5);
  kindr::RotationQuaternionPD rotationStartToReference(kindr::EulerAnglesZyxPD(M_PI/5, -M_PI/6, -5*M_PI/4));

  // Linear velocity aligned with x-axis of reference frame and arbitrary magnitude
  kindr::Velocity3D linearVelocityReferenceToStartInStartFrame(3.4 * kindr::RotationMatrixPD(rotationStartToReference).matrix().row(0).transpose());

  // Angular velocity aligned with z-axis of reference frame and arbitrary magnitude
  kindr::LocalAngularVelocityD angularVelocityReferenceToStartInStartFrame(0.8 * kindr::RotationMatrixPD(rotationStartToReference).matrix().row(2).transpose());

  // Time for a rotation angle of pi
  double timeHorizon = M_PI/angularVelocityReferenceToStartInStartFrame.norm();

  kindr::HomTransformQuatD poseStartToReference(positionReferenceToStartInReferenceFrame, rotationStartToReference);

  // Final approximated pose
  kindr::Position3D approximatedPostitionReferenceToEndInReferenceFrame = positionReferenceToStartInReferenceFrame
      + linearVelocityReferenceToStartInStartFrame.norm() * timeHorizon * kindr::Position3D(-1.0, 0.0, 0.0);
  kindr::RotationQuaternionPD realRotationEndToReference(kindr::EulerAnglesZyxPD(M_PI/5 + M_PI, -M_PI/6, -5*M_PI/4));
  kindr::HomTransformQuatD approximatedPoseEndToReference(approximatedPostitionReferenceToEndInReferenceFrame, realRotationEndToReference);

  kindr::TwistLocalD twistReferenceToStartInStartFrame(linearVelocityReferenceToStartInStartFrame, angularVelocityReferenceToStartInStartFrame);
  kindr::HomTransformQuatD predictedPoseEndToReference = getApproximatedPoseEndToReferenceFromTwistReferenceToStartInStartFrame(
      poseStartToReference,twistReferenceToStartInStartFrame,timeHorizon);

  std::string msg{""};
  KINDR_ASSERT_DOUBLE_MX_EQ(approximatedPoseEndToReference.getTransformationMatrix(), predictedPoseEndToReference.getTransformationMatrix(), 1.0e-10, msg);

};


