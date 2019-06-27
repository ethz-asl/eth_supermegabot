/*!
 * @file     SmbState.cpp
 * @author   Koen Kraemer
 * @date     Aug, 2018
 */

// smb model
#include "smb_model/SmbState.hpp"

// stl
#include <iomanip>


namespace smb_model {

void SmbState::getPoseBaseToWorld(kindr::HomTransformQuatD& poseBaseToWorld) const {
  poseBaseToWorld = kindr::HomTransformQuatD(getPositionWorldToBaseInWorldFrame(), getOrientationBaseToWorld());
}

void SmbState::setPoseBaseToWorld(const kindr::HomTransformQuatD& poseBaseToWorld) {
  setOrientationBaseToWorld(poseBaseToWorld.getRotation());
  setPositionWorldToBaseInWorldFrame(poseBaseToWorld.getPosition());
}

void SmbState::setGeneralizedCoordinatesToLinearlyInterpolated(double t, const SmbState& state0, const SmbState& state1) {
  SmbState res;
  if (t <= 0.0) {
    *this = state0;
    return;
  }
  else if (t >= 1.0) {
    *this = state1;
    return;
  }
  positionWorldToBaseInWorldFrame_ = state0.getPositionWorldToBaseInWorldFrame()* (1.0 - t) + state1.getPositionWorldToBaseInWorldFrame() * t;
  orientationBaseToWorld_ = state0.getOrientationBaseToWorld().boxPlus(state1.getOrientationBaseToWorld().boxMinus(state0.getOrientationBaseToWorld())*t);
  wheelVelocities_ = state0.getWheelVelocities()* (1.0 - t) + state1.getWheelVelocities() * t;
  wheelTorques_ = state0.getWheelTorques()* (1.0 - t) + state1.getWheelTorques() * t;
}

SmbState SmbState::boxPlus(double delta, unsigned int uIndex, bool useQuaternion) const {
  SmbState res = *this;
  switch (uIndex) {
    case static_cast<unsigned int>(RD::GeneralizedVelocitiesEnum::L_X): {
      Position positionWorldToBaseInWorldFrame = positionWorldToBaseInWorldFrame_;
      positionWorldToBaseInWorldFrame(0) += delta;
      res.setPositionWorldToBaseInWorldFrame(positionWorldToBaseInWorldFrame);
    } break;
    case static_cast<unsigned int>(RD::GeneralizedVelocitiesEnum::L_Y): {
      Position positionWorldToBaseInWorldFrame = positionWorldToBaseInWorldFrame_;
      positionWorldToBaseInWorldFrame(1) += delta;
      res.setPositionWorldToBaseInWorldFrame(positionWorldToBaseInWorldFrame);
    } break;
    case static_cast<unsigned int>(RD::GeneralizedVelocitiesEnum::L_Z): {
      Position positionWorldToBaseInWorldFrame = positionWorldToBaseInWorldFrame_;
      positionWorldToBaseInWorldFrame(2) += delta;
      res.setPositionWorldToBaseInWorldFrame(positionWorldToBaseInWorldFrame);
    } break;
    case static_cast<unsigned int>(RD::GeneralizedVelocitiesEnum::A_X): {
      if (useQuaternion) {
        RotationQuaternion orientationWorldToBase = orientationBaseToWorld_.inverted().boxPlus(Eigen::Vector3d(-delta, 0.0, 0.0)).inverted();
        res.setOrientationBaseToWorld(orientationWorldToBase);
      }
      else {
        const kindr::EulerAnglesZyxD eulerAngles(orientationBaseToWorld_);
        res.setOrientationBaseToWorld(kindr::RotationQuaternionD(kindr::EulerAnglesZyxD(eulerAngles.vector() + Eigen::Vector3d(delta, 0.0, 0.0))));
      }
    } break;
    case static_cast<unsigned int>(RD::GeneralizedVelocitiesEnum::A_Y): {
      if (useQuaternion) {
          RotationQuaternion orientationWorldToBase = orientationBaseToWorld_.inverted().boxPlus(Eigen::Vector3d(0.0, -delta, 0.0)).inverted();
          res.setOrientationBaseToWorld(orientationWorldToBase);
      }
      else {
        const kindr::EulerAnglesZyxD eulerAngles(orientationBaseToWorld_);
        res.setOrientationBaseToWorld(kindr::RotationQuaternionD(kindr::EulerAnglesZyxD(eulerAngles.vector() + Eigen::Vector3d(0.0, delta, 0.0))));
      }
    } break;
    case static_cast<unsigned int>(RD::GeneralizedVelocitiesEnum::A_Z): {
      if (useQuaternion) {
          RotationQuaternion orientationWorldToBase = orientationBaseToWorld_.inverted().boxPlus(Eigen::Vector3d(0.0, 0.0, -delta)).inverted();
          res.setOrientationBaseToWorld(orientationWorldToBase);
      }
      else {
        const kindr::EulerAnglesZyxD eulerAngles(orientationBaseToWorld_);
        res.setOrientationBaseToWorld(kindr::RotationQuaternionD(kindr::EulerAnglesZyxD(eulerAngles.vector() + Eigen::Vector3d(0.0, 0.0, delta))));
      }
    } break;
    default: throw std::runtime_error("SmbState::boxPlus Wrong index!");
  }
  return res;
}

//TODO what to do with this?
//const Pose& SmbState::getFrameTransform(const smb_description::SmbTopology::FrameTransformEnum& transformEnum) const {
//  switch (transformEnum) {
//    case smb_description::SmbTopology::FrameTransformEnum::MapToOdom:
//      return poseMapToOdom_;
//    case smb_description::SmbTopology::FrameTransformEnum::MapGaToOdom:
//      return poseMapGaToOdom_;
//  }
//  throw std::runtime_error("SmbState::getFrameTransform: pose does not exist!");
//}

//TODO what to do with this?
//void SmbState::setFrameTransform(const smb_description::SmbTopology::FrameTransformEnum& transformEnum, const Pose& pose) {
//  switch (transformEnum) {
//    case smb_description::SmbTopology::FrameTransformEnum::MapToOdom: {
//      poseMapToOdom_ = pose;
//    } break;
//    case smb_description::SmbTopology::FrameTransformEnum::MapGaToOdom: {
//      poseMapGaToOdom_ = pose;
//    } break;
//    default: throw std::runtime_error("SmbState::getFrameTransform: pose does not exist!");
//  }
//}

const SmbState::WheelVelocities& SmbState::getWheelVelocities() const {
  return wheelVelocities_;
}

SmbState::WheelVelocities& SmbState::getWheelVelocities() {
  return wheelVelocities_;
}

void SmbState::setWheelVelocities(const SmbState::WheelVelocities& wheelVelocities) {
  wheelVelocities_ = wheelVelocities;
}

const SmbState::WheelTorques& SmbState::getWheelTorques() const {
  return wheelTorques_;
}

SmbState::WheelTorques& SmbState::getWheelTorques() {
  return wheelTorques_;
}

void SmbState::setWheelTorques(const SmbState::WheelTorques& wheelTorques) {
  wheelTorques_ = wheelTorques;
}

std::ostream& operator<<(std::ostream& out, const SmbState& state) {
  out << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << std::showpoint << std::right << std::showpos;
  // std::setfill( ' ' )
  out << "Position:";
  out << " x: " << state.positionWorldToBaseInWorldFrame_.x();
  out << " y: " << state.positionWorldToBaseInWorldFrame_.y();
  out << " z: " << state.positionWorldToBaseInWorldFrame_.z();
  out << std::endl;

  out << "Orientation:";
  out << std::endl;
  EulerAnglesZyx eulerAnglesZyx(state.orientationBaseToWorld_);
  eulerAnglesZyx.setUnique();
  out << " EulerZyx: ";
  out << " z: " << eulerAnglesZyx.z();
  out << " y: " << eulerAnglesZyx.y();
  out << " x: " << eulerAnglesZyx.x();
  out << std::endl;

  out << " Quaternion: ";
  out << " w: " << state.orientationBaseToWorld_.w();
  out << " x: " << state.orientationBaseToWorld_.x();
  out << " y: " << state.orientationBaseToWorld_.y();
  out << " z: " << state.orientationBaseToWorld_.z();
  out << std::endl;

  out << "Linear velocity: ";
  out << " x: " << state.linearVelocityBaseInWorldFrame_.x();
  out << " y: " << state.linearVelocityBaseInWorldFrame_.y();
  out << " z: " << state.linearVelocityBaseInWorldFrame_.z();
  out << std::endl;

  out << "Angular velocity:";
  out << " x: " << state.angularVelocityBaseInBaseFrame_.x();
  out << " y: " << state.angularVelocityBaseInBaseFrame_.y();
  out << " z: " << state.angularVelocityBaseInBaseFrame_.z();
  out << std::endl;

  out << "Wheel velocities:";
  out << " LF: " << state.wheelVelocities_(0);
  out << " RF: " << state.wheelVelocities_(1);
  out << " LH: " << state.wheelVelocities_(2);
  out << " RH: " << state.wheelVelocities_(3);
  out << std::endl;

  out << "Wheel torques:";
  out << " LF: " << state.wheelTorques_(0);
  out << " RF: " << state.wheelTorques_(1);
  out << " LH: " << state.wheelTorques_(2);
  out << " RH: " << state.wheelTorques_(3);
  out << std::endl;

  return out;
}


} /* namespace smb_model */
