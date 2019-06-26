/*
 * SmbModel.cpp
 *
 *  Created on: Aug 1, 2018
 *      Author: Koen Kraemer
 */

// smb model
#include <smb_model/SmbModel.hpp>
#include <smb_model/common/rbdl_utils.hpp>

// romo
#include <romo_rbdl/rbdl_utils.hpp>
#include <romo_rbdl/container_utils.hpp>
#include <romo_std/common/container_utils.hpp>

// stl
#include <fstream>

// message logger
#include <message_logger/message_logger.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// robot utils
#include <robot_utils/math/LinearAlgebra.hpp>
#include <robot_utils/physical_definitions.hpp>


namespace smb_model {

inline bool fileExists(const std::string& pathToFile)
{
  return (bool)std::ifstream(pathToFile);
}

SmbModel::SmbModel(double dt)
    : Base(),
      isRealRobot_(false),
      timeStep_(dt),
      useQuaternion_(true)
{

}

const Eigen::MatrixXd& SmbModel::getActuatorSelectionMatrix() const {
  return actuatorSelectionMatrix_;
}

void SmbModel::setIsRealRobot(bool isRealRobot) {
  isRealRobot_ = isRealRobot;
}

bool SmbModel::getIsRealRobot() const {
  return isRealRobot_;
}

//TODO check where this is called and if smb_model_description is used
bool SmbModel::initModelFromUrdfString(const std::string& urdfDescription, bool useQuaternion, bool verbose) {
  useQuaternion_ = useQuaternion;

//TODO joints should not be necessary
  // Create the ordered sequence of joint names from the topology
  std::vector<std::string> joints(0);
  // for (const auto& jointKey: RD::getJointKeys()) {
  //   joints[jointKey.getId()] = jointKey.getName();
  // }

  // Initialize the model
  rbdlModel_.reset(new RigidBodyDynamics::Model());
  if (!romo::URDFReadFromString(urdfDescription.c_str(), rbdlModel_.get(), joints, verbose, useQuaternion, robot_utils::physical_definitions::getAbsoluteGravityAcceleration())) {
    MELO_WARN_STREAM("[SmbModel::initModelFromUrdfString] Error parsing URDF description.");
    return false;
  }

  return initializeModel(useQuaternion);
}

//TODO check where this is called and if smb_model_description is used
bool SmbModel::initModelFromUrdfFile(const std::string& pathToUrdfFile, bool useQuaternion, bool verbose)
{
  useQuaternion_ = useQuaternion;

  // Search for URDF file
  if (!fileExists(pathToUrdfFile)) {
    MELO_WARN_STREAM("[SmbModel::initModelFromUrdfFile] Could not find URDF file '" << pathToUrdfFile << "': file does not exist.");
    return false;
  }

//TODO joints should not be necessary
  // Create the ordered sequence of joint names from the topology
  std::vector<std::string> joints(0);
  // for (const auto& jointKey: RD::getJointKeys()) {
  //   joints[jointKey.getId()] = jointKey.getName();
  // }

  // Initialize the model
  rbdlModel_.reset(new RigidBodyDynamics::Model());
  if (!romo::URDFReadFromFile(pathToUrdfFile.c_str(), rbdlModel_.get(), joints, verbose, useQuaternion, robot_utils::physical_definitions::getAbsoluteGravityAcceleration())) {
    MELO_WARN_STREAM("[SmbModel::initModelFromUrdfFile] Error parsing URDF file '" << pathToUrdfFile << "': file does not exist.");
    return false;
  }

  return initializeModel(useQuaternion);
}

bool SmbModel::initializeModel(bool useQuaternion) {
  // Initialize the containers
  //TODO how is this going to work for smb?
  romo_rbdl::fillBodyContainer<ConcreteDescription>(bodyContainer_, rbdlModel_);
  romo_std::fillBranchNodeShPtrContainer<ConcreteDescription>(bodyBranchNodeContainer_, bodyContainer_);
  romo_rbdl::fillContactContainer<ConcreteDescription, SmbState>(contactContainer_, this);

  // Set the initial state to zero
  setRbdlQFromState(stateGeneralizedPositionsQuaternionRBDL_, state_);
  setRbdlQDotFromState(stateGeneralizedVelocitiesAngularRBDL_, state_);
  updateKinematics(true, true, false);

  // Initialize kinematic parameters
  Eigen::Vector3d position;

//TODO should probably delete
  // Loop over all limbs to set parameters.
  // for (const auto& limbKey : RD::getLimbKeys()) {
  //   const auto limb = limbKey.getEnum();
  //   /* main body */
  //   position = getPositionBodyToBody(
  //       BodyEnum::BASE, smb_description::SmbDefinitions::mapLimbToHip::at(limb),
  //       CoordinateFrameEnum::BASE);
  //   params_.setPositionBaseToHipInBaseFrame(Position(position), limb);

  //   /* hip */
  //   position = getPositionBodyToBody(
  //       smb_description::SmbDefinitions::mapLimbToHip::at(limb),
  //       smb_description::SmbDefinitions::mapLimbToThigh::at(limb),
  //       CoordinateFrameEnum::BASE);
  //   params_.setPositionHipToThighInHipFrame(Position(position), limb);

  //   /* thigh */
  //   position = getPositionBodyToBody(
  //       smb_description::SmbDefinitions::mapLimbToThigh::at(limb),
  //       smb_description::SmbDefinitions::mapLimbToShank::at(limb),
  //       CoordinateFrameEnum::BASE);
  //   params_.setPositionThighToShankInThighFrame(Position(position), limb);

  //   /* shank */
  //   position = getPositionBodyToBody(
  //       smb_description::SmbDefinitions::mapLimbToShank::at(limb),
  //       smb_description::SmbDefinitions::mapLimbToFoot::at(limb),
  //       CoordinateFrameEnum::BASE);
  //   params_.setPositionShankToFootInShankFrame(Position(position), limb);
  // }

//TODO ActuatorDimension == 0 so there should not even need to be a selectionMatrix
  actuatorSelectionMatrix_ = Eigen::Matrix<double, RD::getNumDof(), RD::getNumDof()>::Zero();

  // check for physical validity of the model
  return Base::validateModelParameters();
}

// TODO can indeed be left out?
// const JointTorques& SmbModel::getJointTorques() const {
//   return state_.getJointTorques();
// }

// void SmbModel::setJointTorques(const JointTorques& jointTorques) {
//   state_.setJointTorques(jointTorques);
// }

double SmbModel::getTimeStep() const {
  return timeStep_;
}

void SmbModel::addVariablesToLog(bool update, const std::string& ns) {
  signal_logger::add(state_.getPositionWorldToBaseInWorldFrame(),"posWorldToBaseInWorldFrame",ns);
  signal_logger::add(state_.getOrientationBaseToWorld(),"orientBaseToWorldQuat",ns);

  signal_logger::add(state_.getLinearVelocityBaseInWorldFrame(),"linVelBaseInWorldFrame",ns);
  signal_logger::add(state_.getAngularVelocityBaseInBaseFrame(), "angVelBaseInBaseFrame",ns);
}

void SmbModel::setState(const SmbState& state, bool updatePosition, bool updateVelocity, bool updateAcceleration) {
  state_ = state;
  setRbdlQFromState(stateGeneralizedPositionsQuaternionRBDL_, state_);
  setRbdlQDotFromState(stateGeneralizedVelocitiesAngularRBDL_, state_);
  updateKinematics(updatePosition, updateVelocity, updateAcceleration);
}

} /* namespace smb_model */
