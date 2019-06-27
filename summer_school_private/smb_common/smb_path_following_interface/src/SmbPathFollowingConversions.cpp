//
// Created by johannes on 13.06.19.
//
#include <smb_path_following_interface/SmbPathFollowingConversions.h>

using namespace smb_path_following;

void SmbPathFollowingConversions::writeMpcState(SmbPathFollowingConversions::state_vector_t& stateVector,
                                                const kindr::HomTransformQuatD& pose) {
  // write x and y position into the first 2 elements
  stateVector.head<2>() = pose.getPosition().toImplementation().head<2>();
  // convert current quaternion to Euler angles and use the yaw rotation
  kindr::EulerAnglesRpyD rpy(pose.getRotation());

  kindr::Velocity3D v1(1, 0, 0);
  kindr::Velocity3D v2 = pose.getRotation().rotate(v1);
  stateVector[2] = std::atan2(v2(1), v2(0));
}

void SmbPathFollowingConversions::readMpcState(const SmbPathFollowingConversions::state_vector_t& stateVector,
                                               kindr::HomTransformQuatD& pose) {
  kindr::Position3D position;
  position.x() = stateVector[0];
  position.y() = stateVector[1];
  position.z() = 0;
  kindr::EulerAnglesRpyD rotation;
  rotation.setYaw(stateVector[2]);

  pose.getPosition() = position;
  pose.getRotation() = kindr::RotationQuaternionD(rotation);
}

void SmbPathFollowingConversions::readMpcInput(const SmbPathFollowingConversions::input_vector_t& inputVector, kindr::TwistLocalD& twist) {
  kindr::Velocity3D linearVelocity;
  linearVelocity(0) = inputVector[0];

  kindr::LocalAngularVelocityD angularVelocity;
  angularVelocity(2) = inputVector[1];

  twist.getTranslationalVelocity() = linearVelocity;
  twist.getRotationalVelocity() = angularVelocity;
}

void SmbPathFollowingConversions::writeMpcObservation(Observation& observation, const any_measurements::Pose& pose) {
  writeMpcState(observation.state(), pose.pose_);
  observation.time() = pose.time_.toSeconds();
}
