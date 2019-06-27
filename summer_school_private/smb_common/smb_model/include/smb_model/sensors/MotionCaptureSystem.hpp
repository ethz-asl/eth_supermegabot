/*!
 * @file 	MotionCaptureSystem.hpp
 * @author 	Christian Gehring
 * @date 	  Dec 2014
 * @version 1.0
 * @ingroup robot_model
 */

#pragma once

#include <Eigen/Core>

namespace quadruped_model {

class MotionCaptureSystem {
  friend class Sensors;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotionCaptureSystem():
    isSimulatingMocap_(false),
    timeStamp_(0.0),
    translation_(Eigen::Vector3d::Zero()),
    rotation_(Eigen::Quaterniond(1,0,0,0)),
    translationStd_(0.0),
    rotationStd_(0.0),
    q_CB_(Eigen::Quaterniond(1,0,0,0)),
    B_r_B_C_(Eigen::Vector3d::Zero()),
    q_OI_(Eigen::Quaterniond(1,0,0,0)),
    I_r_I_O_(Eigen::Vector3d::Zero())
    {

  }
  virtual ~MotionCaptureSystem(){};

  const Eigen::Vector3d& getTranslation() const {
    return translation_;
  }

  Eigen::Vector3d& getTranslation() {
    return translation_;
  }

  void setTranslation(Eigen::Vector3d translation) {
    translation_ = translation;
  }

  const Eigen::Quaterniond& getRotation() const {
    return rotation_;
  }

  Eigen::Quaterniond& getRotation() {
    return rotation_;
  }

  void setRotation(const Eigen::Quaterniond& rotation) {
    rotation_ = rotation;
  }

  bool isSimulated() const {
    return isSimulatingMocap_;
  }

  double& getTimeStamp() {
    return timeStamp_;
  }
  void setTimeStamp(double timeStamp) {
    timeStamp_ = timeStamp;
  }


  const Eigen::Quaterniond& getRotationWorldToMarker() const {
    return q_OI_;
  }

  const Eigen::Quaterniond& getRotationBaseToMarker() const {
    return q_CB_;
  }

  const Eigen::Vector3d& getPositionWorldToMarkerInWorldFrame() const {
    return I_r_I_O_;
  }

  const Eigen::Vector3d& getPositionBaseToMarkerInBaseFrame() const {
    return B_r_B_C_;
  }
protected:
  //! if true, mocap is simulated
  bool isSimulatingMocap_;

  //! Timestamp of the actual optitrack data
  double timeStamp_;

  //! Estimated translation of the marker frame w.r.t the optitrack inertial frame
  Eigen::Vector3d translation_;

  //! Estimated rotation of the marker frame w.r.t the optitrack inertial frame
  Eigen::Quaterniond rotation_;

  //! Standard deviation of optitrack position estimate
  double translationStd_;

  //! Standard deviation of optitrack rotation estimate
  double rotationStd_;

  //! Rotation from body frame to optitrack marker frame (used for simulation)
  Eigen::Quaterniond q_CB_;

  //! Translation from body frame to optitrack marker expressed in body frame (used for simulation)
  Eigen::Vector3d B_r_B_C_;

  //! Rotation from inertial frame to optitrack frame (used for simulation)
  Eigen::Quaterniond q_OI_;

  //! Translation from inertial frame to optitrack frame expressed in inertial frame (used for simulation)
  Eigen::Vector3d I_r_I_O_;
protected:
};

} // namespace quadruped_model
