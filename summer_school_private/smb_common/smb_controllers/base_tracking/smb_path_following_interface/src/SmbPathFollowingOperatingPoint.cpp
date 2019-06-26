//
// Created by johannes on 02.05.19.
//
#include <smb_path_following_interface/SmbPathFollowingOperatingPoint.h>

using namespace smb_path_following;

void SmbPathFollowingOperatingPoint::getSystemOperatingTrajectories(const state_vector_t& initialState, const scalar_t& startTime,
                                                    const scalar_t& finalTime, scalar_array_t& timeTrajectory,
                                                    state_vector_array_t& stateTrajectory, input_vector_array_t& inputTrajectory,
                                                    bool concatOutput ) {
  if (concatOutput == false) {
    timeTrajectory.clear();
    stateTrajectory.clear();
    inputTrajectory.clear();
  }

  timeTrajectory.emplace_back(startTime);
  timeTrajectory.emplace_back(finalTime);

  stateTrajectory.emplace_back(initialState);
  stateTrajectory.emplace_back(initialState);

  inputTrajectory.emplace_back(input_vector_t::Zero());
  inputTrajectory.emplace_back(input_vector_t::Zero());
}
