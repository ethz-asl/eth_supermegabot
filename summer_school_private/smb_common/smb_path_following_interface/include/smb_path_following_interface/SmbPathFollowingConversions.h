//
// Created by johannes on 13.06.19.
//

#pragma once

#include <smb_path_following_interface/SmbPathFollowingExplicitTemplateInstantiations.h>
#include <any_measurements/Pose.hpp>
#include <any_measurements/Twist.hpp>
#include <kindr/Core>

namespace smb_path_following {

class SmbPathFollowingConversions {
 public:
  typedef ocs2::SystemObservation<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_> Observation;
  typedef ocs2::MPC_Interface<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_> MpcInterface;
  typedef MpcInterface::state_vector_t state_vector_t;
  typedef MpcInterface::input_vector_t input_vector_t;

  static void writeMpcState(state_vector_t& stateVector, const kindr::HomTransformQuatD& pose);
  static void readMpcState(const state_vector_t& stateVector, kindr::HomTransformQuatD& pose);

  static void readMpcInput(const input_vector_t& inputVector, kindr::TwistLocalD& twist);

  static void writeMpcObservation(Observation& observation, const any_measurements::Pose& pose);
};

}  // namespace smb_path_following