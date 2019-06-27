//
// Created by johannes on 02.05.19.
//
#include <smb_path_following_interface/SmbPathFollowingExplicitTemplateInstantiations.h>


template class ocs2::MPC_SLQ<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
template class ocs2::ConstraintBase<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
template class ocs2::CostFunctionBase<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
template class ocs2::RobotInterfaceBase<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
template class ocs2::Dimensions<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
template class ocs2::SystemOperatingPoint<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
template class ocs2::MPC_Interface<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
template class ocs2::SystemObservation<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
template class ocs2::CostDesiredTrajectories<ocs2::Dimensions<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>::scalar_t>;
