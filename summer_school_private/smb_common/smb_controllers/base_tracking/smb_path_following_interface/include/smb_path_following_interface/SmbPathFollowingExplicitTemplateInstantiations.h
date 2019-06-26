//
// Created by johannes on 02.05.19.
//

#pragma once

#include <ocs2_core/misc/loadEigenMatrix.h>
#include <ocs2_comm_interfaces/SystemObservation.h>
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h>
#include <ocs2_core/Dimensions.h>
#include <ocs2_mpc/MPC_SLQ.h>
#include <ocs2_robotic_tools/common/RobotInterfaceBase.h>
#include "smb_path_following_interface/SmbPathFollowingDefinitions.h"
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>


extern template class ocs2::MPC_SLQ<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
extern template class ocs2::ConstraintBase<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
extern template class ocs2::CostFunctionBase<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
extern template class ocs2::RobotInterfaceBase<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
extern template class ocs2::Dimensions<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
extern template class ocs2::SystemOperatingPoint<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
extern template class ocs2::MPC_Interface<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>;
extern template class ocs2::CostDesiredTrajectories<ocs2::Dimensions<smb_path_following::STATE_DIM_, smb_path_following::INPUT_DIM_>::scalar_t>;
