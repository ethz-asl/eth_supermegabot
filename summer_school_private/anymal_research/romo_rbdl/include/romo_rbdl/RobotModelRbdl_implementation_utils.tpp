/*
 * RobotModelRbdl_implementation_utils.tpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Dario Bellicoso
 */

#include "romo_rbdl/rbdl_kinematics.hpp"
#include "romo_rbdl/rbdl_utils.hpp"
#include "romo_rbdl/container_utils.hpp"
#include "romo_rbdl/RobotModelRbdl.hpp"

#include "romo_std/common/container_utils.hpp"

#include <romo/CollisionsRomoUtils.hpp>

namespace romo_rbdl {

template<typename ConcreteDescription_, typename RobotState_>
void RobotModelRbdl<ConcreteDescription_, RobotState_>::updateKinematics(
      bool updatePosition,
      bool updateVelocity,
      bool updateAcceleration) {
  if (!updatePosition && !updateVelocity && !updateAcceleration) {
    return;
  }
  if (updatePosition && updateVelocity && updateAcceleration) {
    throw std::runtime_error("RobotModel::updateKinematics(): Update accelerations is not supported!");
    return;
  }


  if (updatePosition && updateVelocity) {
    RigidBodyDynamics::UpdateKinematicsCustom(*rbdlModel_.get(),
                                              &stateGeneralizedPositionsQuaternionRBDL_,
                                              &stateGeneralizedVelocitiesAngularRBDL_,
                                              nullptr);
    return;
  }
  if (updatePosition) {
    RigidBodyDynamics::UpdateKinematicsCustom(*rbdlModel_.get(),
                                              &stateGeneralizedPositionsQuaternionRBDL_,
                                              nullptr,
                                              nullptr);
    return;
  }
  if (updateVelocity) {
    RigidBodyDynamics::UpdateKinematicsCustom(*rbdlModel_.get(),
                                              nullptr,
                                              &stateGeneralizedVelocitiesAngularRBDL_,
                                              nullptr);
    return;
  }
  if (updateAcceleration) {
    throw std::runtime_error("RobotModel::updateKinematics(): Update accelerations is not supported!");
  }
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::validateModelParameters() const {

  bool isValid = true;

  for (const auto& body : this->bodyContainer_) {

    if (!rbdlModel_->IsFixedBodyId(body->getBodyId())) {

      /************************
       * Test inertia tensors *
       ************************/
      const Eigen::MatrixXd& inertiaMatrix = body->getInertiaMatrix();

      // Retreive the inertia tensor provided by the model
      const double Ixx = inertiaMatrix(0,0);
      const double Iyy = inertiaMatrix(1,1);
      const double Izz = inertiaMatrix(2,2);
      const double Ixy = inertiaMatrix(0,1);
      const double Ixz = inertiaMatrix(0,2);
      const double Iyz = inertiaMatrix(1,2);

      // Extract principle moments of inertia
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(inertiaMatrix,Eigen::EigenvaluesOnly);
      const Eigen::VectorXd eigenValues = eigenSolver.eigenvalues();
      const double Ix = eigenValues[0];
      const double Iy = eigenValues[1];
      const double Iz = eigenValues[2];

      // test symmetry
      if (!(inertiaMatrix.transpose()-inertiaMatrix).isZero()) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " is not symmetric!");
        isValid = false;
      }

      // test eigenvalues
      if ((eigenValues.array() <= 0.0).any()) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " is not positive definite! Eigenvalues are: " << eigenValues.transpose());
        MELO_WARN_STREAM("[RobotModel::validateModel] Name of body in model: " << rbdlModel_->GetBodyName(body->getBodyId()));
        MELO_WARN_STREAM("[RobotModel::validateModel] Body id: " << body->getBodyId());
        isValid = false;
      }

      // test triangle inequality
      if ( !(Ix+Iy>Iz) &&
           !(std::abs((Ix+Iy) - Iz) < 1.0e-3) ) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " does not respect the triangle inequality! Ix+Iy<Iz! "
                         << "Ix=" << Ix << ", Iy=" << Iy << ", Iz=" << Iz << " error: " << std::abs((Ix+Iy) - Iz));
        isValid = false;
      }
      if (!(Ix+Iz>Iy) && !(std::abs((Ix+Iz) - Iy) < 1.0e-3)) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " does not respect the triangle inequality! Ix+Iz<Iy! "
                         << "Ix=" << Ix << ", Iy=" << Iy << ", Iz=" << Iz << " error: " << std::abs((Ix+Iz) - Iy));
        isValid = false;
      }
      if (!(Iy+Iz>Ix) && !(std::abs((Iy+Iz) - Ix) < 1.0e-3)) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " does not respect the triangle inequality! Iy+Iz<Ix! "
                         << "Ix=" << Ix << ", Iy=" << Iy << ", Iz=" << Iz << " error: " << std::abs((Iy+Iz) - Ix));
        isValid = false;
      }

      // test the product of inertia
      if (!(Ixx > 2.0*std::fabs(Iyz))
       && !(std::abs(Ixx - 2.0*std::abs(Iyz)) < 1.0e-5) ) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " does not respect the product of inertia inequality! Ixx < 2.0*|Iyz|!");
        isValid = false;
      }
      if (!(Iyy >= 2.0*std::fabs(Ixz))) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " does not respect the product of inertia inequality! Iyy < 2.0*|Ixz|!");
        isValid = false;
      }
      if (!(Izz >= 2.0*std::fabs(Ixy))) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " does not respect the product of inertia inequality! Izz < 2.0*|Ixy|!");
        isValid = false;
      }
      if (!(Ixx*Iyy >= Ixy*Ixy)) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " does not respect the product of inertia inequality! Ixx*Iyy < Ixy*Ixy!");
        isValid = false;
      }
      if (!(Ixx*Izz >= Ixz*Ixz)) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " does not respect the product of inertia inequality! Ixx*Izz < Ixz*Ixz!");
        isValid = false;
      }
      if (!(Iyy*Izz >= Iyz*Iyz)) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Inertia tensor of body " << body->getName() << " does not respect the product of inertia inequality! Iyy*Izz < Iyz*Iyz!");
        isValid = false;
      }

      /*************
       * Test mass *
       *************/
      if ( body->getMass() < 0.0 ) {
        MELO_WARN_STREAM("[RobotModel::validateModel] Mass of body " << body->getName() << " is not positive!");
        isValid = false;
      }
    }

  }

  return isValid;
}


template<typename ConcreteDescription_, typename RobotState_>
void RobotModelRbdl<ConcreteDescription_, RobotState_>::printModelParameters() const {

  MELO_INFO_STREAM("[RobotModel::printModelParameters] Printing model parameters");

  for (const auto it : rbdlModel_->mBodyNameMap) {
    MELO_INFO_STREAM("[RobotModel::printModelParameters] -------------------");
    MELO_INFO_STREAM("[RobotModel::printModelParameters] Found body: " << it.first);
    if (it.second < rbdlModel_->fixed_body_discriminator) {
      MELO_INFO_STREAM("[RobotModel::printModelParameters] Movable body.");
      MELO_INFO_STREAM("[RobotModel::printModelParameters] Id: " << it.second);
      MELO_INFO_STREAM("[RobotModel::printModelParameters] mass: " << rbdlModel_->mBodies[it.second].mMass);
      MELO_INFO_STREAM("[RobotModel::printModelParameters] com: " << rbdlModel_->mBodies[it.second].mCenterOfMass.transpose());
    } else {
      const int fixedBodyId = it.second-rbdlModel_->fixed_body_discriminator;
      const int movableParentId = rbdlModel_->mFixedBodies[fixedBodyId].mMovableParent;
      MELO_INFO_STREAM("[RobotModel::printModelParameters] Fixed body.");
      MELO_INFO_STREAM("[RobotModel::printModelParameters] Id: " << fixedBodyId);
      MELO_INFO_STREAM("[RobotModel::printModelParameters] mass: " << rbdlModel_->mFixedBodies[fixedBodyId].mMass);
      MELO_INFO_STREAM("[RobotModel::printModelParameters] com: " << rbdlModel_->mFixedBodies[fixedBodyId].mCenterOfMass.transpose());
      MELO_INFO_STREAM("[RobotModel::printModelParameters] Movable parent: " << rbdlModel_->GetBodyName(movableParentId));
    }
    MELO_INFO_STREAM("[RobotModel::printModelParameters] -------------------");
  }

}


template<typename ConcreteDescription_, typename RobotState_>
void RobotModelRbdl<ConcreteDescription_, RobotState_>::printModelHierarchy() const
{
  if (rbdlModel_) {
    std::cout << "Model Hierarchy:\n" << RigidBodyDynamics::Utils::GetModelHierarchy(*rbdlModel_) << std::endl;
    std::cout << "Model DOF Overview:\n" << RigidBodyDynamics::Utils::GetModelDOFOverview(*rbdlModel_) << std::endl;
    std::cout << "Named Body Origins Overview:\n" << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(*rbdlModel_) << std::endl;
  } else {
    std::cout << "[RobotModel::printModelHierarchy] Could not print robot model: model was not initialized" << std::endl;
  }
}

template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::initializeFromUrdf(const std::string &urdfString, bool verbose) {

  // Initialize the model
  rbdlModel_.reset(new RigidBodyDynamics::Model());

  // Create the ordered sequence of joint names from the topology
  std::vector<std::string> joints(RD::getJointsDimension());
  for( const auto &jointIt : RD::getJointKeys() ) {
    joints[jointIt.getId()] = jointIt.getName();
  }

  // Check for suffix to decide parsing
  const std::string suffix = ".urdf";
  urdf::ModelInterfaceSharedPtr urdfModel;
  if( urdfString.size() >= suffix.size() && urdfString.compare(urdfString.size() - suffix.size(), suffix.size(), suffix) == 0 ) {
    // Initialize romo from a urdf file
    if( !romo::URDFReadFromFile(urdfModel, urdfString.c_str(), rbdlModel_.get(), joints, verbose) ) {
      MELO_ERROR_STREAM("[RobotModel::initializeFromUrdf] Error parsing URDF from File.");
      return false;
    }
    MELO_INFO_STREAM("[RobotModel:initializeFromUrdf]: Parsed URDF for romo from file!");
  } else {
    // Initialize romo from a urdf string
    if( !romo::URDFReadFromString(urdfModel, urdfString.c_str(), rbdlModel_.get(), joints, verbose) ) {
      MELO_ERROR_STREAM("[RobotModel::initializeFromUrdf] Error parsing URDF from String.");
      return false;
    }
    MELO_INFO_STREAM("[RobotModel:initializeFromUrdf]: Parsed URDF for romo from string!");
  }

  // Initialize the containers
  romo_rbdl::fillBodyContainer<ConcreteDescription_>(this->bodyContainer_, rbdlModel_);
  romo_std::fillBranchNodeShPtrContainer<ConcreteDescription_>(this->bodyBranchNodeContainer_, this->bodyContainer_);
  romo_rbdl::fillContactContainer<ConcreteDescription_, RobotState_>(this->contactContainer_, this);

  // Validate parameters
  bool validParameters = validateModelParameters();
  if( !validParameters ) { MELO_WARN_STREAM("[RobotModel::initializeFromUrdf] Model parameters are not valid."); }

  bool collisionsGeometrySet = romo::setCollisionsGeometryFromModel(urdfModel,this->bodyContainer_);
  if( !collisionsGeometrySet ) { MELO_WARN_STREAM("[RobotModel::initializeFromUrdf] Could not set collisions geometry."); }

  return validParameters && collisionsGeometrySet;
}

} /* namespace romo */
