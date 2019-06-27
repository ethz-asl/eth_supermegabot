/*
 * rbdl_utils.h
 *
 *  Created on: Oct 1, 2015
 *      Author: Dario Bellicoso, Koen Kraemer
 */

#pragma once


// smb model
#include "smb_model/SmbState.hpp"

// std_utils
#include "std_utils/std_utils.hpp"

// eigen
#include <Eigen/Core>

namespace smb_model {

//! Enum RBDL generalized coordinates vector
CONSECUTIVE_ENUM(GeneralizedCoordinatesRbdlEnum,
                 X, Y, Z, Q_X, Q_Y, Q_Z, Q_W);

void setRbdlQFromState(Eigen::VectorXd& rbdlQ, const SmbState& state);
void setRbdlQDotFromState(Eigen::VectorXd& rbdlQDot, const SmbState& state);
void setStateFromRbdlQ(SmbState& state, const Eigen::VectorXd& rbdlQ);

} // smb_model
