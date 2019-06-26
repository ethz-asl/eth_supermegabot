
#include <romo/RobotModel.hpp>

namespace romo {


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModel<ConcreteDescription_, RobotState_>::getLimbJointPositionsFromContactEnumIteratively (
  Eigen::VectorXd& limbJointPositions,
  const Eigen::Vector3d& desiredPositionBaseToTargetPointInBaseFrame,
  const ContactEnum contactEnum, 
  const double toleranceForFailCheck, 
  const double toleranceForIteration,
  const unsigned int maxIterations,
  const bool verbose) 
{
  const auto bodyEnum = RD::template mapEnums<BodyEnum>(contactEnum);
  return getLimbJointPositionsFromPositionBaseToTargetPointInBaseFrameIteratively(limbJointPositions, desiredPositionBaseToTargetPointInBaseFrame, bodyEnum,
   Eigen::Vector3d::Zero(), toleranceForFailCheck, toleranceForIteration, maxIterations, verbose);
}

template<typename ConcreteDescription_, typename RobotState_>
bool RobotModel<ConcreteDescription_, RobotState_>::getLimbJointPositionsFromLimbEnumIteratively (
    Eigen::VectorXd& limbJointPositions,
    const Eigen::Vector3d& desiredPositionBaseToTargetPointInBaseFrame,
    const LimbEnum limbEnum, 
    const double toleranceForFailCheck, 
    const double toleranceForIteration,
    const unsigned int maxIterations,
    const bool verbose) 
{
  const auto branchEnum = RD::template mapEnums<BranchEnum>(limbEnum);
  const auto limbEndBody = RD::getBranchEndBody(branchEnum);
  return getLimbJointPositionsFromPositionBaseToTargetPointInBaseFrameIteratively(limbJointPositions, desiredPositionBaseToTargetPointInBaseFrame, limbEndBody,
   Eigen::Vector3d::Zero(), toleranceForFailCheck, toleranceForIteration, maxIterations, verbose);  
}

template<typename ConcreteDescription_, typename RobotState_>
bool RobotModel<ConcreteDescription_, RobotState_>::getLimbJointPositionsFromPositionBaseToTargetPointInBaseFrameIteratively(
    Eigen::VectorXd& limbJointPositions,
    const Eigen::Vector3d& desiredPositionBaseToTargetPointInBaseFrame,
    const BodyEnum bodyEnum,
    const Eigen::Vector3d& positionBodyToTargetPointInBodyFrame,
    const double toleranceForFailCheck,
    const double toleranceForIteration,
    const unsigned int maxIterations,
    const bool verbose,
    std::function<void(JointPositions&, const Eigen::VectorXd&, const LimbEnum)> updateJoints)
{
//  bool converged = false;

  constexpr auto numDofTranslational = RD::getNumTranslationalDof();
  const auto bodyNodeEnum = RD::template mapEnums<BodyNodeEnum>(bodyEnum);
  const auto limbEnum = RD::template mapEnums<LimbEnum>(bodyEnum);
  const auto branchEnum = RD::template mapEnums<BranchEnum>(bodyEnum);
  const auto numDofInvKin = RD::getNumDofLimb(limbEnum);
  const auto branchIdU = RD::getBranchStartIndexInU(RD::template mapEnums<BranchEnum>(limbEnum));
  const auto limbIdJ = RD::getLimbStartIndexInJ(limbEnum);

  //initialize actuator positions
  RobotState robotStateInvKin = getState();
  Eigen::MatrixXd jacobianTranslationFloatingBaseToPoinOnBody = Eigen::MatrixXd::Zero(
      numDofTranslational, RD::getNumDof());
  Eigen::Matrix<double, numDofTranslational, 1> positionErrorInBaseFrame;  //position error in B system: actual - desired
  JointPositions newJointPositions = robotStateInvKin.getJointPositions();

  // Copy state since we are changing it.
  const JointPositions jointPositionsCopy = robotStateInvKin.getJointPositions();

  // Iterate.
  unsigned int k = 0;
  for (; k < maxIterations; ++k) {

    positionErrorInBaseFrame = desiredPositionBaseToTargetPointInBaseFrame
        - (getBody(branchEnum, bodyNodeEnum).getPositionWorldToPointOnBody(
            positionBodyToTargetPointInBodyFrame, RD::CoordinateFrameEnum::BASE)
            - getBody(RD::BodyEnum::BASE).getPositionWorldToBody(RD::CoordinateFrameEnum::BASE));

    // Check if solution has converged.
    if (positionErrorInBaseFrame.norm() <= toleranceForIteration) {
//      converged = true;
      break;
    }

    getJacobianTranslationFloatingBaseToPointOnBody(jacobianTranslationFloatingBaseToPoinOnBody,
                                                    positionBodyToTargetPointInBodyFrame,
                                                    branchEnum, bodyNodeEnum,
                                                    RD::CoordinateFrameEnum::BASE);

    const Eigen::MatrixXd jacobianTranslationFloatingBaseToPoinOnBodyInverse =
        robot_utils::pseudoInverseAdaptiveDls(
            jacobianTranslationFloatingBaseToPoinOnBody.middleCols(branchIdU, numDofInvKin));
    const Eigen::VectorXd jointUpdate = jacobianTranslationFloatingBaseToPoinOnBodyInverse
        * positionErrorInBaseFrame;

    // Update state.
    updateJoints(newJointPositions, jointUpdate, limbEnum);

    if (verbose) {
      std::cout << k << ": " << positionErrorInBaseFrame.norm() << std::endl;
    }
    robotStateInvKin.setJointPositions(newJointPositions);
    setState(robotStateInvKin, true, false);
    if (verbose) {
      std::cout << getState().getJointPositions() << std::endl;
    }
  }

  // Copy result, even if not converged.
  limbJointPositions = getState().getJointPositions().toImplementation().template segment(
      limbIdJ, numDofInvKin);

  // Revert to original state.
  robotStateInvKin.setJointPositions(jointPositionsCopy);
  setState(robotStateInvKin, true, false);

  // Check if tolerance reached.
  if (positionErrorInBaseFrame.norm() > toleranceForFailCheck) {
    if (verbose) {
      std::cout << "[" << RD::getLimbKeys().at(limbEnum).getName()
          << "]: Did not find a valid solution for iterative inverse kinematics. Remaining position error in base frame: "
          << positionErrorInBaseFrame.norm() << "." << std::endl;
      std::cout << "Robot state: " << std::endl << getState() << std::endl;
      std::cout << "Desired endeffector position: "
                << desiredPositionBaseToTargetPointInBaseFrame.transpose() << std::endl;
    }
    return false;
  } else {
    if (verbose) {
      std::cout << "[" << RD::getLimbKeys().at(limbEnum).getName()
          << "]: Found valid solution within " << k
          << " iterations. Remaining position error in base frame: "
          << positionErrorInBaseFrame.norm() << "." << std::endl;
    }
    return true;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModel<ConcreteDescription_, RobotState_>::getLimbJointPositionsFromPoseEndEffectorToBaseIteratively(
    Eigen::VectorXd& limbJointPositions,
    const Pose& desiredPoseEndEffectorToBase,
    const LimbEnum limbEnum,
    const bool doReconfiguration,
    const Eigen::VectorXd* desiredJoints,
    const double updateGain,
    const double toleranceForFailCheck, 
    const double toleranceForIteration,
    const unsigned int maxIterations,
    const bool verbose) 
{
  const auto numDofLimb = RD::getNumDofLimb(limbEnum);
  constexpr auto numDofSpatial = RD::getNumSpatialDof();

  RobotState robotStateBeforeIk = getState();
  RobotState robotStateCopy = getState();

  // Initialize joint positions.
  JointPositions jointPositions = robotStateCopy.getJointPositions();

  const auto branchEnum = RD::template mapEnums<BranchEnum>(limbEnum);
  const auto endEffector = RD::getBranchEndBody(branchEnum);
  const auto endEffectorNode = RD::template mapEnums<BodyNodeEnum>(endEffector);
//  const auto limbId = RD::mapKeyEnumToKeyId(limbEnum);
  const auto branchIdU = RD::getBranchStartIndexInU(branchEnum);
  const auto limbIdJ = RD::getLimbStartIndexInJ(limbEnum);

  // Check input.
  if (doReconfiguration) {
    if (desiredJoints == nullptr) {
      std::cout << "[RobotModel::getLimbJointPositionsFromPoseEndEffectorToBaseIteratively] desiredJoints size was nullptr!" << std::endl;
      return false;
    }
    if (desiredJoints->size() != numDofLimb) {
      std::cout << "[RobotModel::getLimbJointPositionsFromPoseEndEffectorToBaseIteratively] desiredJoints size was different from numDofLimb!" << std::endl;
      return false;
    }
  }

  Eigen::MatrixXd spatialJacobianWorldToEndEffectorInWorldFrame = Eigen::MatrixXd::Zero(numDofSpatial, RD::getNumDof());
  Eigen::Matrix<double, numDofSpatial, 1> spatialErrorInWorldFrame = Eigen::Matrix<double, numDofSpatial, 1>::Zero();

  auto k = 0u;
  for(; k < maxIterations; ++k) {
    // Forward kinematics.
    Eigen::Vector3d positionBaseToEndEffectorInBaseFrame(
        getPositionBodyToBody(RD::BodyEnum::BASE, endEffector, RD::CoordinateFrameEnum::BASE));

    const Eigen::Matrix3d& orientationWorldToBase(getOrientationWorldToBody(RD::BodyEnum::BASE));
    const Eigen::Matrix3d& orientationWorldToEndEffector(getOrientationWorldToBody(endEffector));

    const RotationQuaternion orientationWorldToDesiredEndEffector =
        desiredPoseEndEffectorToBase.getRotation().inverted()
            * RotationQuaternion(RotationMatrix(orientationWorldToBase));

    spatialErrorInWorldFrame.template head<3>() = - orientationWorldToEndEffector.transpose()*(orientationWorldToDesiredEndEffector.
        boxMinus(RotationQuaternion(RotationMatrix(orientationWorldToEndEffector))));
    spatialErrorInWorldFrame.template tail<3>() = orientationWorldToBase.transpose()*
        (desiredPoseEndEffectorToBase.getPosition().toImplementation()
            - positionBaseToEndEffectorInBaseFrame);

    if (spatialErrorInWorldFrame.norm() < toleranceForIteration) {
      break;
    }

    // Get jacobian.
    getJacobianSpatialWorldToBody(
        spatialJacobianWorldToEndEffectorInWorldFrame,
        branchEnum, endEffectorNode, RD::CoordinateFrameEnum::WORLD);
    const Eigen::MatrixXd spatialJacobianBaseToEndEffectorInWorldFrame =
        spatialJacobianWorldToEndEffectorInWorldFrame.middleCols(branchIdU, numDofLimb);

    const Eigen::MatrixXd spatialJacobianBaseToEndEffectorInWorldFrameInverted =
        robot_utils::pseudoInverseAdaptiveDls(spatialJacobianBaseToEndEffectorInWorldFrame);

    // Gains and jacobian inverse/transpose.
    if (doReconfiguration) {
      Eigen::VectorXd q_null = -updateGain * (jointPositions.toImplementation().segment(limbIdJ, numDofLimb) - *desiredJoints);
      Eigen::VectorXd nullSpaceUpdate = (Eigen::MatrixXd::Identity(numDofLimb, numDofLimb)
          - spatialJacobianBaseToEndEffectorInWorldFrameInverted * spatialJacobianWorldToEndEffectorInWorldFrame.middleCols(branchIdU, numDofLimb)) * q_null;
      jointPositions.toImplementation().segment(limbIdJ, numDofLimb) += nullSpaceUpdate;
    }

    jointPositions.toImplementation().segment(limbIdJ, numDofLimb) +=
        updateGain * (spatialJacobianBaseToEndEffectorInWorldFrameInverted * spatialErrorInWorldFrame);
    robotStateCopy.setJointPositions(jointPositions);
    setState(robotStateCopy, true, false, false);
  }

  // Update limb joint positions.
  limbJointPositions = jointPositions.toImplementation().segment(limbIdJ, numDofLimb);

  // Set back the original state.
  setState(robotStateBeforeIk, true, true, false);

  // Check if tolerance reached.
  if(spatialErrorInWorldFrame.norm() > toleranceForFailCheck) {
    if (verbose) {
      std::cout << "[" << RD::getLimbKeys().at(limbEnum).getName() << "]: Found no valid solution within " << k << " iterations. Remaining error: " << spatialErrorInWorldFrame.norm() << ", update gain: " << updateGain << "." << std::endl;
    }
    return false;
  }

  return true;
}

} // namespace
