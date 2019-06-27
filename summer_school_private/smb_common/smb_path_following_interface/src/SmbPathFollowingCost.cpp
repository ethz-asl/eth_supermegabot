//
// Created by johannes on 01.05.19.
//
#include <smb_path_following_interface/SmbPathFollowingCost.h>

using namespace smb_path_following;

template <typename SCALAR_T>
void SmbPathFollowingCost::intermediateCostFunction(const SCALAR_T& time, const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
                                                const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::INPUT_DIM_, 1>& input,
                                                const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
                                                const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::REFERENCE_INPUT_DIM_, 1>& inputDesired,
                                                const Eigen::Matrix<SCALAR_T, BASE::logic_variable_dim_, 1>& logicVariable,
                                                SCALAR_T& costValue) {
  // tracking configuration for base
  Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::STATE_DIM_, 1> xDeviationTrackingConfiguration = state - stateDesired;

  xDeviationTrackingConfiguration(2) = sin(0.5 * (stateDesired(2) - state(2)));

  SCALAR_T controlCost = 0.5 * input.dot(RJointTracking_.template cast<SCALAR_T>().lazyProduct(input));
  SCALAR_T stateCost = 0.5 * xDeviationTrackingConfiguration.dot(
                                 QTrackingConfiguration_.template cast<SCALAR_T>().lazyProduct(xDeviationTrackingConfiguration));
  costValue = stateCost + controlCost;
}

template <typename SCALAR_T>
void SmbPathFollowingCost::terminalCostFunction(const SCALAR_T& time, const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
                                            const Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
                                            const Eigen::Matrix<SCALAR_T, BASE::logic_variable_dim_, 1>& logicVariable,
                                            SCALAR_T& costValue) {
  // tracking configuration for base
  Eigen::Matrix<SCALAR_T, SmbPathFollowingDefinitions::STATE_DIM_, 1> xDeviationTrackingConfiguration = state - stateDesired;

  xDeviationTrackingConfiguration(2) = sin(0.5 * (stateDesired(2) - state(2)));

  costValue = 0.5 * xDeviationTrackingConfiguration.dot(
                        QTrackingConfigurationFinal_.template cast<SCALAR_T>().lazyProduct(xDeviationTrackingConfiguration));
}

// explicit instantiations:

template void SmbPathFollowingCost::intermediateCostFunction<SmbPathFollowingCost::ad_base_t>(
    const ad_base_t& time, const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
    const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::INPUT_DIM_, 1>& input,
    const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
    const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::REFERENCE_INPUT_DIM_, 1>& inputDesired,
    const Eigen::Matrix<ad_base_t, BASE::logic_variable_dim_, 1>& logicVariable, ad_base_t& costValue);

template void SmbPathFollowingCost::intermediateCostFunction<SmbPathFollowingCost::scalar_t>(
    const scalar_t& time, const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
    const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::INPUT_DIM_, 1>& input,
    const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
    const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::REFERENCE_INPUT_DIM_, 1>& inputDesired,
    const Eigen::Matrix<scalar_t, BASE::logic_variable_dim_, 1>& logicVariable, scalar_t& costValue);

template void SmbPathFollowingCost::terminalCostFunction<SmbPathFollowingCost::scalar_t>(
    const scalar_t& time, const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
    const Eigen::Matrix<scalar_t, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
    const Eigen::Matrix<scalar_t, BASE::logic_variable_dim_, 1>& logicVariable, scalar_t& costValue);

template void SmbPathFollowingCost::terminalCostFunction<SmbPathFollowingCost::ad_base_t>(
    const ad_base_t& time, const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::STATE_DIM_, 1>& state,
    const Eigen::Matrix<ad_base_t, SmbPathFollowingDefinitions::REFERENCE_STATE_DIM_, 1>& stateDesired,
    const Eigen::Matrix<ad_base_t, BASE::logic_variable_dim_, 1>& logicVariable, ad_base_t& costValue);
