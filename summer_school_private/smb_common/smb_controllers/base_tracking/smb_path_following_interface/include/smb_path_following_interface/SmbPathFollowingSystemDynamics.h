//
// Created by johannes on 29.04.19.
//

#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>

#include <cppad/cg.hpp>
#include "smb_path_following_interface/SmbPathFollowingDefinitions.h"

namespace smb_path_following {

class SmbPathFollowingSystemDynamics : public ocs2::SystemDynamicsBaseAD<SmbPathFollowingSystemDynamics, SmbPathFollowingDefinitions::STATE_DIM_, SmbPathFollowingDefinitions::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<SmbPathFollowingSystemDynamics> Ptr;
  typedef std::shared_ptr<const SmbPathFollowingSystemDynamics> ConstPtr;

  typedef ocs2::SystemDynamicsBaseAD<SmbPathFollowingSystemDynamics, SmbPathFollowingDefinitions::STATE_DIM_, SmbPathFollowingDefinitions::INPUT_DIM_> BASE;
  typedef typename BASE::scalar_t scalar_t;
  typedef typename BASE::state_vector_t state_vector_t;
  typedef typename BASE::state_matrix_t state_matrix_t;
  typedef typename BASE::input_vector_t input_vector_t;
  typedef typename BASE::state_input_matrix_t state_input_matrix_t;

  typedef CppAD::AD<CppAD::cg::CG<scalar_t>> ad_base_t;

  /**
   * Constructor
   *
   * @param [in] dynamicLibraryIsCompiled: Whether the library has been already complied.
   */
  SmbPathFollowingSystemDynamics(const bool& dynamicLibraryIsCompiled = false) : BASE(dynamicLibraryIsCompiled) {}

  /**
   * Destructor
   */
  ~SmbPathFollowingSystemDynamics() = default;

  /**
   * Interface method to the state flow map of the hybrid system. This method should be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector
   * @param [out] stateDerivative: state vector time derivative.
   */
  template <typename SCALAR_T>
  void systemFlowMap(const SCALAR_T& time, const Eigen::Matrix<SCALAR_T, BASE::state_dim_, 1>& state,
                     const Eigen::Matrix<SCALAR_T, BASE::input_dim_, 1>& input,
                     Eigen::Matrix<SCALAR_T, BASE::state_dim_, 1>& stateDerivative);
};

extern template void SmbPathFollowingSystemDynamics::systemFlowMap<SmbPathFollowingSystemDynamics::ad_base_t>(
    const ad_base_t& time, const Eigen::Matrix<ad_base_t, BASE::state_dim_, 1>& state,
    const Eigen::Matrix<ad_base_t, BASE::input_dim_, 1>& input, Eigen::Matrix<ad_base_t, BASE::state_dim_, 1>& stateDerivative);

extern template void SmbPathFollowingSystemDynamics::systemFlowMap<SmbPathFollowingSystemDynamics::scalar_t>(const scalar_t& time,
                                                                             const Eigen::Matrix<scalar_t, BASE::state_dim_, 1>& state,
                                                                             const Eigen::Matrix<scalar_t, BASE::input_dim_, 1>& input,
                                                                             Eigen::Matrix<scalar_t, BASE::state_dim_, 1>& stateDerivative);

}  // namespace smb_path_following
