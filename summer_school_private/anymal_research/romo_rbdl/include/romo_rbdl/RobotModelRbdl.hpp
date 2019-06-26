/*
 * RobotModelRbdl.hpp
 *
 *  Created on: Mar 9, 2016
 *      Author: dbellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>

// std
#include <memory>

// any_rbdl
#include <any_rbdl/rbdl.h>

// romo
#include "romo/RobotState.hpp"
#include "romo/RobotModel.hpp"
#include "romo_rbdl/containers.hpp"
#include "romo_std/common/containers.hpp"


namespace romo_rbdl {

template<typename ConcreteDescription_, typename RobotState_>
class RobotModelRbdl : public romo::RobotModel<ConcreteDescription_, RobotState_> {
 private:
  //! Helper alias for the base class.
  using Base = romo::RobotModel<ConcreteDescription_, RobotState_>;

 public:
  using typename Base::RD;
  using typename Base::BodyEnum;
  using typename Base::BodyNodeEnum;
  using typename Base::CoordinateFrameEnum;
  using typename Base::BranchEnum;
  using typename Base::LimbEnum;
  using typename Base::RobotState;
  using ContactEnum = typename RD::ContactEnum;
  using ContactState = typename RD::ContactStateEnum;

 public:
  RobotModelRbdl() = default;
  ~RobotModelRbdl() override = default;

  bool initializeFromUrdf(const std::string &urdfString, bool verbose);

  void updateKinematics(
      bool updatePosition = true,
      bool updateVelocity = false,
      bool updateAcceleration = false) override;

  /*! @returns the number of degrees of freedom of the robot */
  unsigned int getDofCount() const override { return rbdlModel_->dof_count; }

  bool getPositionWorldToBody(
      Eigen::Vector3d& position,
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionWorldToBody(
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const override;

  bool getPositionWorldToBody(
      Eigen::Vector3d& position,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionWorldToBody(
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getPositionWorldToPointOnBody(
      Eigen::Vector3d& position,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionWorldToPointOnBody(
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const override;

  bool getPositionWorldToPointOnBody(
      Eigen::Vector3d& position,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionWorldToPointOnBody(
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getPositionWorldToBodyCom(
      Eigen::Vector3d& position,
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionWorldToBodyCom(
      BodyEnum bodyEnum,
      CoordinateFrameEnum frame) const override;

  bool getPositionWorldToBodyCom(
      Eigen::Vector3d& position,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionWorldToBodyCom(
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getPositionBodyToBody(
      Eigen::Vector3d& position,
      BodyEnum fromBody,
      BodyEnum toBody,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionBodyToBody(
      BodyEnum fromBody,
      BodyEnum toBody,
      CoordinateFrameEnum frame) const override;

  bool getPositionBodyToBody(
      Eigen::Vector3d& position,
      BodyEnum fromBody,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionBodyToBody(
      BodyEnum fromBody,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const override;

  bool getPositionBodyToBody(
      Eigen::Vector3d& position,
      BranchEnum fromBranch,
      BodyNodeEnum fromNode,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionBodyToBody(
      BranchEnum fromBranch,
      BodyNodeEnum fromNode,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const override;

  bool getPositionBodyToBodyCom(
      Eigen::Vector3d& position,
      BodyEnum fromBody,
      BodyEnum toBody,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionBodyToBodyCom(
      BodyEnum fromBody,
      BodyEnum toBody,
      CoordinateFrameEnum frame) const override;

  bool getPositionBodyToBodyCom(
      Eigen::Vector3d& position,
      BodyEnum fromBody,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionBodyToBodyCom(
      BodyEnum fromBody,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const override;

  bool getPositionBodyToBodyCom(
      Eigen::Vector3d& position,
      BranchEnum fromBranch,
      BodyNodeEnum fromNode,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionBodyToBodyCom(
      BranchEnum fromBranch,
      BodyNodeEnum fromNode,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const override;

  bool setPositionBodyToBodyCom(
      const Eigen::Vector3d& position,
      BodyEnum body,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionBaseToLimbComInBaseFrame(BranchEnum branch) const override;

  void getPositionWorldToCom(
      Eigen::VectorXd& positionWorldToCom,
      CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getPositionWorldToCom(
      CoordinateFrameEnum frame) const override;
  /************************/

  /************************
   * Get orientation methods *
   ************************/

  const Eigen::Matrix3d& getOrientationWorldToBody(BodyEnum bodyEnum) const override;

  const Eigen::Matrix3d& getOrientationWorldToBody(
      BranchEnum branch,
      BodyNodeEnum bodyNode) const override;

  const Eigen::Matrix3d getOrientationBodyToBody(
      BodyEnum fromBody,
      BodyEnum toBody) const override;

  const Eigen::Matrix3d getOrientationBodyToBody(
      BranchEnum fromBranch,
      BodyNodeEnum fromBodyNode,
      BranchEnum toBranch,
      BodyNodeEnum toBodyNode) const override;

  /************************/

  /************************
   * Get Jacobian methods *
   ************************/
   bool getJacobianSpatialWorldToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianSpatialWorldToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianSpatialTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianSpatialTimeDerivativeWorldToBody,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianSpatialTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianTimeDerivativeWorldToBody,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianSpatialTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianTimeDerivativeWorldToBody,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianSpatialTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianTimeDerivativeWorldToBody,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationWorldToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationWorldToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationWorldToBodyCom(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationWorldToCom(
      Eigen::MatrixXd& jacobian,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianTranslationTimeDerivative,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianTranslationTimeDerivative,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianTranslationTimeDerivative,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianTranslationTimeDerivative,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationTimeDerivativeWorldToCom(
      Eigen::MatrixXd& jacobianTranslationTimeDerivativeWorldToCom,
      CoordinateFrameEnum frame) const override;

  bool getJacobianRotationWorldToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianRotationWorldToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianRotationTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianRotationTimeDerivative,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianRotationTimeDerivativeWorldToBody(
      Eigen::MatrixXd& jacobianRotationTimeDerivative,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianRotationTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianRotationTimeDerivative,
      const Eigen::MatrixXd& jacobianSpatialWorldToBodyInWorldFrame,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianRotationTimeDerivativeWorldToPointOnBody(
      Eigen::MatrixXd& jacobianRotationTimeDerivative,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  /************************
   * Get Floating Base Jacobian methods *
   ************************/

  bool getJacobianTranslationFloatingBaseToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationFloatingBaseToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianTranslationFloatingBaseToBodyCom(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianRotationFloatingBaseToBody(
      Eigen::MatrixXd& jacobian,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getJacobianRotationFloatingBaseToPointOnBody(
      Eigen::MatrixXd& jacobian,
      const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;
  /************************/

  /************************
   * Get manipulability measures *
   ************************/
  double getManipulabilityMeasure(
      BranchEnum branch,
      BodyNodeEnum bodyNode) const override;

  void getManipulabilityMeasureGradientWorldToBody(
      Eigen::VectorXd& manipulabilityMeasureGradient,
      BranchEnum branch,
      BodyNodeEnum bodyNode) const override;

  void getManipulabilityMeasureGradientBodyToBody(
      Eigen::VectorXd& manipulabilityMeasureGradient,
      BranchEnum fromBranch,
      BodyNodeEnum fromBodyNode,
      BranchEnum toBranch,
      BodyNodeEnum toBodyNode) const override;

  /************************/


  /************************
   * Get Hessian methods *
   ************************/

  bool getHessianSpatialWorldToBodyForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getHessianTranslationWorldToBodyForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;

  bool getHessianTranslationWorldToBodyForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame,
      Eigen::MatrixXd& spatialJacobian) const override;

  bool getHessianTranslationWorldToComForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      CoordinateFrameEnum frame) const override;

  bool getHessianRotationWorldToBodyForState(
      Eigen::MatrixXd& hessian,
      unsigned int qIndex,
      BranchEnum branch,
      BodyNodeEnum node,
      CoordinateFrameEnum frame) const override;
  /************************/

  /************************
   * Get velocity methods *
   ************************/
  Eigen::Vector3d getLinearVelocityWorldToBody(
    BodyEnum body,
    CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getLinearVelocityWorldToBody(
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getLinearVelocityWorldToPointOnBody(
    BodyEnum body,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getLinearVelocityWorldToPointOnBody(
    BranchEnum branch,
    BodyNodeEnum node,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getLinearVelocityBodyToPointOnBody(
    BodyEnum fromBody,
    BodyEnum toBody,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getLinearVelocityWorldToBodyCom(
      BodyEnum body, CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getLinearVelocityWorldToCom(CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getAngularVelocityWorldToBody(
    BodyEnum body,
    CoordinateFrameEnum frame) const override;

  Eigen::Vector3d getAngularVelocityWorldToBody(
    BranchEnum branch,
    BodyNodeEnum bodyNode,
    CoordinateFrameEnum frame) const override;
  /************************/

  /************
   * Dynamics *
   ************/

  double getGravityAcceleration() const override;
  const Eigen::Vector3d& getGravityVectorInWorldFrame() const override;

  double getBodyMass(BodyEnum body) const override;
  double getBodyMass(BranchEnum branch, BodyNodeEnum node) const override;
  bool setBodyMass(BodyEnum body, double mass) const override;

  double getRootMass() const override;

  double getLimbMass(BranchEnum branch) const override;

  double getTotalMass() const override;

  bool updateBodyInertia(BodyEnum body) const override;

  const Eigen::Matrix3d& getBodyInertiaMatrix(BodyEnum body) const override;
  bool setBodyInertiaMatrix(BodyEnum body, const Eigen::Matrix3d& inertiaMatrix) const override;

  bool getGravityTerms(Eigen::VectorXd& gravity) const override;
  Eigen::VectorXd getGravityTerms() const override;

  bool getNonlinearEffects(Eigen::VectorXd& nonlinearEffects) const override;
  Eigen::VectorXd getNonlinearEffects() const override;

  bool getMassInertiaMatrix(Eigen::MatrixXd& massMatrix) const override;
  Eigen::MatrixXd getMassInertiaMatrix() const override;

  bool getGravityDynamicParameters(Eigen::VectorXd& parameters) const override;

  bool getJointGravityTermsFromJointConfigurationAndLinkParameters(
    Eigen::VectorXd& gravityTerms, const Eigen::VectorXd& jointPositions,
    const Eigen::VectorXd& dynamicParams) override;

  /************************/

  const romo::RigidBody<ConcreteDescription_>& getBody(BodyEnum body) const override;
  const romo::RigidBody<ConcreteDescription_>& getBody(BranchEnum branch, BodyNodeEnum node) const override;

  /*! @returns a reference to an instance of an RBDL Model object
   */
  RigidBodyDynamics::Model& getRbdlModel() { return *rbdlModel_; }

  /*! Check the model parameters for physical meaning
   * @returns true if successful
   */
  bool validateModelParameters() const;

  void printModelHierarchy() const;
  void printModelParameters() const;

  virtual const RobotState& getState() const override { return state_; }

  const Eigen::VectorXd& getStateGeneralizedPositionsQuaternionRBDL() const { return stateGeneralizedPositionsQuaternionRBDL_; }
  const Eigen::VectorXd& getStateGeneralizedVelocitiesAngularRBDL() const { return stateGeneralizedVelocitiesAngularRBDL_; }


 protected:
  /*******************
   * State variables *
   *******************/
  /*
   * q = [ I_p_IB
   *       q_BI_xyz
   *       q_j
   *       q_BI_w ]
   *
   * dq = [ I_v_B
   *        B_w_B
   *        dq_j]
   */

  //! State of the robot
  RobotState state_;

  std::shared_ptr<RigidBodyDynamics::Model> rbdlModel_;

  //! State of the robot
//  RobotState state_;
  //! these are the state variables used by RBDL.
  Eigen::VectorXd stateGeneralizedPositionsQuaternionRBDL_;
  //! these are the state variables used by RBDL.
  Eigen::VectorXd stateGeneralizedVelocitiesAngularRBDL_;
};

} /* namespace romo */


#include <romo_rbdl/RobotModelRbdl_implementation_position.tpp>
#include <romo_rbdl/RobotModelRbdl_implementation_jacobian.tpp>
#include <romo_rbdl/RobotModelRbdl_implementation_hessian.tpp>
#include <romo_rbdl/RobotModelRbdl_implementation_velocity.tpp>
#include <romo_rbdl/RobotModelRbdl_implementation_dynamics.tpp>
#include <romo_rbdl/RobotModelRbdl_implementation_utils.tpp>
