/*
 * rbdl_utils.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: Dario Bellicoso, Koen Kraemer
 */


// smb model
#include <smb_model/common/rbdl_utils.hpp>

namespace smb_model {

void setRbdlQFromState(Eigen::VectorXd& rbdlQ, const SmbState& state) {
    rbdlQ.resize(state.getNumberOfGeneralizedCoordinates());
    rbdlQ.segment<Position::Dimension>(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::X)) =
        state.getPositionWorldToBaseInWorldFrame().toImplementation();
    auto& orientation = state.getOrientationBaseToWorld();
    rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_X)) = orientation.x();
    rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Y)) = orientation.y();
    rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Z)) = orientation.z();
    rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_W)) = orientation.w();
}

void setRbdlQDotFromState(Eigen::VectorXd& rbdlQDot, const SmbState& state) {
    rbdlQDot.resize(state.getNumberOfGeneralizedCoordinates());
    rbdlQDot = state.getGeneralizedVelocities();
}

void setStateFromRbdlQ(SmbState& state, const Eigen::VectorXd& rbdlQ) {
//    state.setJointPositions(JointPositions(rbdlQ.segment<RD::getJointsDimension()>(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::LF_HAA))));
    state.setPositionWorldToBaseInWorldFrame(Position(rbdlQ.segment<Position::Dimension>(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::X))));
    state.setOrientationBaseToWorld(
        RotationQuaternion(rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_W)),
                           rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_X)),
                           rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Y)),
                           rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Z))));
}

} // namespace
