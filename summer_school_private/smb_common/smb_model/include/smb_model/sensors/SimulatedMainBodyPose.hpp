/*!
 * @file 	SimulatedMainBodyPose.hpp
 * @author 	Christian Gehring
 * @date 	  Dec 2014
 * @version 1.0
 * @ingroup quadruped_model
 */

#pragma once


namespace quadruped_model {

class SimulatedMainBodyPose {
  friend class Sensors;
public:
  SimulatedMainBodyPose() {
    I_r_IB_.setZero();
    I_v_B_.setZero();
    I_a_B_.setZero();
    rpy_BI_.setZero();
    drpy_BI_.setZero();
    I_omega_IB_.setZero();
    I_psi_IB_.setZero();

    B_omega_IB_.setZero();
    B_v_B_.setZero();

    I_r_IB_std_.setZero();
    I_v_B_std_.setZero();
    I_a_B_std_.setZero();
    rpy_BI_std_.setZero();
    drpy_BI_std_.setZero();
    ddrpy_BI_std_.setZero();
  };
  virtual ~SimulatedMainBodyPose() {};

  quadruped_model::VectorQb getQb() {
    quadruped_model::VectorQb qb;
    qb.block<3,1>(0,0) =  I_r_IB_;
    qb.block<3,1>(3,0) = rpy_BI_;
    return qb;
  }

  void setQb(const quadruped_model::VectorQb& Qb) {
    I_r_IB_ = Qb.block<3,1>(0,0);
    rpy_BI_ = Qb.block<3,1>(3,0);
  }

  quadruped_model::VectorQb getdQb() {
    quadruped_model::VectorQb dqb;
    dqb.block<3,1>(0,0) =  I_v_B_;
    dqb.block<3,1>(3,0) = drpy_BI_;
    return dqb;
  }

  void setdQb(const quadruped_model::VectorQb& dQb) {
    I_v_B_ = dQb.block<3,1>(0,0);
    drpy_BI_ = dQb.block<3,1>(3,0);
  }

  void setLinearVelocityBaseInWorldFrame(const Eigen::Vector3d& I_v_B) {
    I_v_B_ = I_v_B;
  }

  const Eigen::Vector3d& getLinearVelocityBaseInWorldFrame() const {
    return I_v_B_;
  }

  Eigen::Vector3d& getLinearVelocityBaseInWorldFrame() {
    return I_v_B_;
  }


  void setPositionWorldToBaseInWorldFrame(const Eigen::Vector3d& positionWorldToBaseInWorldFrame) {
    I_r_IB_ = positionWorldToBaseInWorldFrame;
  }
  void setOrientationWorldToBaseEulerAnglesXyz(const Eigen::Vector3d& xyz) {
    rpy_BI_ = xyz;
  }
  void setEulerAnglesXyzDiff(const Eigen::Vector3d& xyzDiff) {
    drpy_BI_ = xyzDiff;
   }

  void setLinearVelocityBaseInBaseFrame(const Eigen::Vector3d& B_v_B) {
    B_v_B_ = B_v_B;
  }

  const Eigen::Vector3d& getLinearVelocityBaseInBaseFrame() const {
    return B_v_B_;
  }

  Eigen::Vector3d& getLinearVelocityBaseInBaseFrame() {
    return B_v_B_;
  }

  void setAngularVelocityBaseInWorldFrame(const Eigen::Vector3d& I_omega_IB) {
    I_omega_IB_ = I_omega_IB;
  }

  const Eigen::Vector3d& getAngularVelocityBaseInWorldFrame() const {
    return I_omega_IB_;
  }

  Eigen::Vector3d& getAngularVelocityBaseInWorldFrame() {
    return I_omega_IB_;
  }


  void setAngularVelocityBaseInBaseFrame(const Eigen::Vector3d& B_omega_IB) {
    B_omega_IB_ = B_omega_IB;
  }

  const Eigen::Vector3d& getAngularVelocityBaseInBaseFrame() const {
    return B_omega_IB_;
  }

  Eigen::Vector3d& getAngularVelocityBaseInBaseFrame() {
    return B_omega_IB_;
  }


  void setLinearAccelerationBaseInWorldFrame(const Eigen::Vector3d& I_a_B) {
    I_a_B_ = I_a_B;
  }

  const Eigen::Vector3d& getLinearAccelerationBaseInWorldFrame() const {
    return I_a_B_;
  }

  Eigen::Vector3d& getLinearAccelerationBaseInWorldFrame(){
    return I_a_B_;
  }

  void setddQb(const quadruped_model::VectorQb& ddQb) {
    I_a_B_ = ddQb.block<3,1>(0,0);
    // TODO ddrpy not yet implemented
  }

  quadruped_model::VectorQb getddQb() {
    quadruped_model::VectorQb ddqb = quadruped_model::VectorQb::Zero();
    ddqb.block<3,1>(0,0) =  I_a_B_;
    ddqb.block<3,1>(3,0) = Eigen::Vector3d(nan(""),nan(""),nan(""));
    // TODO: ddrpy not yet implemented!!
//    ddqb.block<3,1>(3,0) = ddrpy_BI_;
    return ddqb;
  }

protected:
  //! main body position [x; y; z]
  Eigen::Vector3d I_r_IB_;
  //! main body velocity [x; y; z]
  Eigen::Vector3d I_v_B_;
  //! linear acceleration of the main body
  Eigen::Vector3d I_a_B_;
  //! orientation of the main body (Kardan angles) [roll; pitch; yaw]
  Eigen::Vector3d rpy_BI_;
  //! time derivatives of Kardan angles
  Eigen::Vector3d drpy_BI_;
  Eigen::Vector3d ddrpy_BI_;
  //! angular velocity of the main body
  Eigen::Vector3d I_omega_IB_;
  //! angular acceleration of the main body
  Eigen::Vector3d I_psi_IB_;

  //! angular velocity of the main body
  Eigen::Vector3d B_omega_IB_;
  Eigen::Vector3d B_v_B_;


  //! Standard deviation of base minimal coordinate simulation measurements
  Eigen::Vector3d I_r_IB_std_;
  Eigen::Vector3d I_v_B_std_;
  Eigen::Vector3d I_a_B_std_;

  Eigen::Vector3d rpy_BI_std_;
  Eigen::Vector3d drpy_BI_std_;
  Eigen::Vector3d ddrpy_BI_std_;
};


} // namespace quadruped_model
