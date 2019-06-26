#include <utility>

/*
 * WacoWeightedCost.h
 *
 *  Created on: May 1, 2019
 *      Author: Johannes Pankert
 */

#pragma once

#include <ocs2_core/cost/CostFunctionBaseAD.h>
#include "smb_path_following_interface/SmbPathFollowingDefinitions.h"

#include <cmath>

// CPPAD stuff
#include "cppad/cg/math.hpp"

namespace smb_path_following {

class SmbPathFollowingCost : public ocs2::CostFunctionBaseAD<SmbPathFollowingCost, SmbPathFollowingDefinitions::STATE_DIM_, SmbPathFollowingDefinitions::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<SmbPathFollowingCost> Ptr;
  typedef std::shared_ptr<const SmbPathFollowingCost> ConstPtr;

  typedef CostFunctionBaseAD<SmbPathFollowingCost, SmbPathFollowingDefinitions::STATE_DIM_, SmbPathFollowingDefinitions::INPUT_DIM_> BASE;
  typedef typename BASE::scalar_t scalar_t;
  typedef typename BASE::state_vector_t state_vector_t;
  typedef typename BASE::state_desired_vector_t state_desired_vector_t;
  typedef typename BASE::state_matrix_t state_matrix_t;
  typedef typename BASE::input_vector_t input_vector_t;
  typedef typename BASE::input_matrix_t input_matrix_t;

  typedef CppAD::AD<CppAD::cg::CG<scalar_t>> ad_base_t;

  /**
   * Constructor
   */
  SmbPathFollowingCost(state_matrix_t QTrackingConfiguration, input_matrix_t RJointTracking, state_matrix_t QTrackingConfigurationFinal,
                   const bool& dynamicLibraryIsCompiled = false)
      : BASE(dynamicLibraryIsCompiled),
        QTrackingConfiguration_(std::move(QTrackingConfiguration)),
        QTrackingConfigurationFinal_(std::move(QTrackingConfigurationFinal)),
        RJointTracking_(std::move(RJointTracking)) {}

  /**
   * Copy constructor
   * @param rhs
   */
  SmbPathFollowingCost(const SmbPathFollowingCost& rhs) = default;

  /**
   * Default destructor
   */
  ~SmbPathFollowingCost() override = default;

  /**
   * Interface method to the intermediate cost function. This method should be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [in] stateDesired: desired state vector.
   * @param [in] inputDesired: desired input vector.
   * @param [in] logicVariable: logic variable vector.
   * @param [out] costValue: cost value.
   */
  template <typename SCALAR_T>
  void intermediateCostFunction(const SCALAR_T& time, const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
                                const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::INPUT_DIM_, 1>& input,
                                const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
                                const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::REFERENCE_INPUT_DIM_, 1>& inputDesired,
                                const Eigen::Matrix<SCALAR_T, BASE::logic_variable_dim_, 1>& logicVariable, SCALAR_T& costValue);

  /**
   * Interface method to the terminal cost function. This method should be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] stateDesired: desired state vector.
   * @param [in] logicVariable: logic variable vector.
   * @param [out] costValue: cost value.
   */
  template <typename SCALAR_T>
  void terminalCostFunction(const SCALAR_T& time, const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
                            const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::STATE_DIM_, 1>& stateDesired,
                            const Eigen::Matrix<SCALAR_T, BASE::logic_variable_dim_, 1>& logicVariable, SCALAR_T& costValue);

  /**
   * Gets a user-defined desired state at the given time.
   *
   * @param [in] t: Current time.
   * @return The desired state at the given time.
   */
  state_desired_vector_t getDesiredState(const scalar_t& t) {
    dynamic_vector_t xNominal;
    BASE::xNominalFunc_.interpolate(t, xNominal);

    if (xNominal.size() > SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_)
      throw std::runtime_error("Desired state size cannot be greater than reference dimension.");
    else {
      state_desired_vector_t desiredState = state_desired_vector_t::Zero();
      desiredState.head(xNominal.size()) = xNominal;
      return desiredState;
    }
  }

  /**
   * Gets a user-defined desired input at the given time.
   *
   * @param [in] t: Current time.
   * @return The desired input at the given time.
   */
  input_desired_vector_t getDesiredInput(const scalar_t& t) {
    dynamic_vector_t uNominal;
    BASE::uNominalFunc_.interpolate(t, uNominal);

    if (uNominal.size() > SmbPathFollowingDefinitions::REFERENCE_INPUT_DIM_)
      throw std::runtime_error("Desired input size cannot be greater than input dimension.");
    else {
      input_desired_vector_t desiredInput = input_desired_vector_t::Zero();
      desiredInput.head(uNominal.size()) = uNominal;
      return desiredInput;
    }
  }

 private:
  state_matrix_t QTrackingConfiguration_;

  input_matrix_t RJointTracking_;

  state_matrix_t QTrackingConfigurationFinal_;
};

extern template void SmbPathFollowingCost::intermediateCostFunction<SmbPathFollowingCost::ad_base_t>(
    const ad_base_t& time, const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
    const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::INPUT_DIM_, 1>& input,
    const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
    const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::REFERENCE_INPUT_DIM_, 1>& inputDesired,
    const Eigen::Matrix<ad_base_t, BASE::logic_variable_dim_, 1>& logicVariable, ad_base_t& costValue);

extern template void SmbPathFollowingCost::intermediateCostFunction<SmbPathFollowingCost::scalar_t>(
    const scalar_t& time, const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
    const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::INPUT_DIM_, 1>& input,
    const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
    const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::REFERENCE_INPUT_DIM_, 1>& inputDesired,
    const Eigen::Matrix<scalar_t, BASE::logic_variable_dim_, 1>& logicVariable, scalar_t& costValue);

extern template void SmbPathFollowingCost::terminalCostFunction<SmbPathFollowingCost::scalar_t>(
    const scalar_t& time, const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
    const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
    const Eigen::Matrix<scalar_t, BASE::logic_variable_dim_, 1>& logicVariable, scalar_t& costValue);

extern template void SmbPathFollowingCost::terminalCostFunction<SmbPathFollowingCost::ad_base_t>(
    const ad_base_t& time, const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
    const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
    const Eigen::Matrix<ad_base_t, BASE::logic_variable_dim_, 1>& logicVariable, ad_base_t& costValue);

}  // namespace smb_path_following
