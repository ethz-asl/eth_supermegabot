//
// Created by johannes on 01.05.19.
//
#include <smb_path_following_interface/SmbPathFollowingSystemDynamics.h>

using namespace smb_path_following;

template <typename SCALAR_T>
void SmbPathFollowingSystemDynamics::systemFlowMap(const SCALAR_T& time, const Eigen::Matrix<SCALAR_T, BASE::state_dim_, 1>& state,
                                   const Eigen::Matrix<SCALAR_T, BASE::input_dim_, 1>& input,
                                   Eigen::Matrix<SCALAR_T, BASE::state_dim_, 1>& stateDerivative) {
  const SCALAR_T& theta = state(2);

  // Polar coordinates with base linear and angular velocities as input
  Eigen::Matrix<SCALAR_T, 3, 2> J_base = Eigen::Matrix<SCALAR_T, 3, 2>::Zero();
  J_base(0, 0) = cos(theta);
  J_base(0, 1) = 0.0;
  J_base(1, 0) = sin(theta);
  J_base(1, 1) = 0.0;
  J_base(2, 0) = 0.0;
  J_base(2, 1) = 1.0;

  // time derivatives of base position and heading
  stateDerivative = J_base * input;
}

template void SmbPathFollowingSystemDynamics::systemFlowMap<SmbPathFollowingSystemDynamics::ad_base_t>(const ad_base_t& time,
                                                                       const Eigen::Matrix<ad_base_t, BASE::state_dim_, 1>& state,
                                                                       const Eigen::Matrix<ad_base_t, BASE::input_dim_, 1>& input,
                                                                       Eigen::Matrix<ad_base_t, BASE::state_dim_, 1>& stateDerivative);

template void SmbPathFollowingSystemDynamics::systemFlowMap<SmbPathFollowingSystemDynamics::scalar_t>(const scalar_t& time,
                                                                      const Eigen::Matrix<scalar_t, BASE::state_dim_, 1>& state,
                                                                      const Eigen::Matrix<scalar_t, BASE::input_dim_, 1>& input,
                                                                      Eigen::Matrix<scalar_t, BASE::state_dim_, 1>& stateDerivative);
