/*
 * FiniteDifferences.hpp
 *
 *  Created on: Jun 29, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>

namespace romo {
namespace finite_differences {

namespace internal {
static constexpr auto deltaH = 1.0e-7;
static constexpr auto deltaT = 1.0e-7;
}

template<typename RobotModelType_>
static void estimateLinearVelocityBody(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::Vector3d& linearVelocity) {
  const auto state = robotModel.getState();
  typename std::remove_const<decltype(state)>::type stateCopy;

  const Eigen::VectorXd qDt_plus = state.getGeneralizedVelocities()*internal::deltaT;

  stateCopy = state;
  for (int i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) { stateCopy = stateCopy.boxPlus(qDt_plus(i), i); }
  robotModel.setState(stateCopy, true, true);
  const Eigen::Vector3d pos_dt_p = robotModel.getPositionWorldToBody(branch, node, frame);

  stateCopy = state;
  for (int i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) { stateCopy = stateCopy.boxPlus(-qDt_plus(i), i); }
  robotModel.setState(stateCopy, true, true);
  const Eigen::Vector3d pos_dt_m = robotModel.getPositionWorldToBody(branch, node, frame);

  linearVelocity = (pos_dt_p - pos_dt_m) / (2.0 * internal::deltaT);

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateTranslationHessianWorldToPointOnBodyForState(
    RobotModelType_& robotModel,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    unsigned int perturbedStateIndex, Eigen::MatrixXd& hessian) {
  const auto state = robotModel.getState();

  Eigen::MatrixXd dpdq_p = hessian;
  Eigen::MatrixXd dpdq_m = hessian;

  robotModel.setState(state.boxPlus(internal::deltaH, perturbedStateIndex), true, true);
  robotModel.getJacobianTranslationWorldToPointOnBody(dpdq_p, positionBodyToPointOnBodyInBodyFrame, branch, node, frame);

  robotModel.setState(state.boxPlus(-internal::deltaH, perturbedStateIndex), true, true);
  robotModel.getJacobianTranslationWorldToPointOnBody(dpdq_m, positionBodyToPointOnBodyInBodyFrame, branch, node, frame);

  hessian = (dpdq_p - dpdq_m) / (2.0 * internal::deltaH);

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateTranslationHessianWorldToBodyForState(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    unsigned int perturbedStateIndex, Eigen::MatrixXd& hessian) {
  estimateTranslationHessianWorldToPointOnBodyForState(robotModel, Eigen::Vector3d::Zero(), branch, node, frame, perturbedStateIndex, hessian);
}

template<typename RobotModelType_>
static void estimateRotationHessianWorldToBodyForState(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    unsigned int perturbedStateIndex, Eigen::MatrixXd& hessian) {
  const auto state = robotModel.getState();

  Eigen::MatrixXd dpdq_p = hessian;
  Eigen::MatrixXd dpdq_m = hessian;

  robotModel.setState(state.boxPlus(internal::deltaH, perturbedStateIndex), true, true);
  robotModel.getJacobianRotationWorldToBody(dpdq_p, branch, node, frame);

  robotModel.setState(state.boxPlus(-internal::deltaH, perturbedStateIndex), true, true);
  robotModel.getJacobianRotationWorldToBody(dpdq_m, branch, node, frame);

  hessian = (dpdq_p - dpdq_m) / (2.0 * internal::deltaH);

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateHessianWorldToComInWorldFrameForState(
    RobotModelType_& robotModel,
    unsigned int perturbedStateIndex,
    Eigen::MatrixXd& hessian) {
  const auto state = robotModel.getState();
  typename std::remove_const<decltype(state)>::type stateTemp;

  Eigen::MatrixXd dpdq_p = hessian;
  Eigen::MatrixXd dpdq_m = hessian;

  stateTemp = state.boxPlus(internal::deltaH, perturbedStateIndex);
  robotModel.setState(stateTemp, true, true);
  robotModel.getJacobianTranslationWorldToCom(dpdq_p, RobotModelType_::CoordinateFrameEnum::WORLD);

  stateTemp = state.boxPlus(-internal::deltaH, perturbedStateIndex);
  robotModel.setState(stateTemp, true, true);
  robotModel.getJacobianTranslationWorldToCom(dpdq_m, RobotModelType_::CoordinateFrameEnum::WORLD);

  hessian = (dpdq_p - dpdq_m) / (2.0 * internal::deltaH);

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateTranslationalJacobianWorldToBody(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();

  for (int i=0; i<state.getNumberOfGeneralizedVelocities(); ++i) {
    // Positive perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(internal::deltaH, i), true, true);

    // Get the position of the body when state has been perturbated with +delta.
    Eigen::Vector3d p_p;
    robotModel.getPositionWorldToBody(p_p, branch, node, RobotModelType_::CoordinateFrameEnum::WORLD);

    // Negative perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(-internal::deltaH, i), true, true);

    // Get the position of the body when state has been perturbated with -delta.
    Eigen::Vector3d p_m;
    robotModel.getPositionWorldToBody(p_m, branch, node, RobotModelType_::CoordinateFrameEnum::WORLD);

    if (frame == RobotModelType_::CoordinateFrameEnum::BASE) {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) = state.getOrientationBaseToWorld().inverseRotate(romo::Vector((p_p - p_m) / (2.0 * internal::deltaH))).toImplementation();
    } else {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) = (p_p - p_m) / (2.0 * internal::deltaH);
    }
  }

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateTranslationalJacobianFloatingBaseToPointOnBody(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  constexpr auto numBaseDof = state.getNumberOfGeneralizedVelocities() - state.getNumberOfJointVelocities();

    for (int i=numBaseDof; i<state.getNumberOfGeneralizedVelocities(); ++i) {
    // Positive perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(internal::deltaH, i), true, true);

    // Get the position of the body when state has been perturbated with +delta.
    Eigen::Vector3d p_p;
    robotModel.getPositionBodyToBody(
        p_p, RobotModelType_::BranchEnum::BASE, RobotModelType_::BodyNodeEnum::BASE,
        branch, node, RobotModelType_::CoordinateFrameEnum::WORLD);
    p_p += robotModel.getOrientationWorldToBody(branch, node).transpose()*positionBodyToPointOnBodyInBodyFrame;

    // Negative perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(-internal::deltaH, i), true, true);

    // Get the position of the body when state has been perturbated with -delta.
    Eigen::Vector3d p_m;
    robotModel.getPositionBodyToBody(
        p_m, RobotModelType_::BranchEnum::BASE, RobotModelType_::BodyNodeEnum::BASE,
        branch, node, RobotModelType_::CoordinateFrameEnum::WORLD);
    p_m += robotModel.getOrientationWorldToBody(branch, node).transpose()*positionBodyToPointOnBodyInBodyFrame;

    if (frame == RobotModelType_::CoordinateFrameEnum::BASE) {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) = state.getOrientationBaseToWorld().inverseRotate(romo::Vector((p_p - p_m) / (2.0 * internal::deltaH))).toImplementation();
    } else {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) = (p_p - p_m) / (2.0 * internal::deltaH);
    }
  }

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateRotationalJacobianFloatingBaseToBody(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  constexpr auto numBaseDof = state.getNumberOfGeneralizedVelocities() - state.getNumberOfJointVelocities();

  for (auto i=numBaseDof; i<state.getNumberOfGeneralizedVelocities(); ++i) {
    // Positive perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(internal::deltaH, i), true, true);

    // Get the position of the body when state has been perturbated with +delta.
    const romo::RotationQuaternion r_p =
        romo::RotationQuaternion(romo::RotationMatrix(
            robotModel.getOrientationWorldToBody(branch, node).transpose()));

    // Negative perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(-internal::deltaH, i), true, true);
    const romo::RotationQuaternion r_m =
        romo::RotationQuaternion(romo::RotationMatrix(
            robotModel.getOrientationWorldToBody(branch, node).transpose()));

    if (frame == RobotModelType_::CoordinateFrameEnum::BASE) {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) = state.getOrientationBaseToWorld().inverseRotate(
          romo::Vector(r_p.boxMinus(r_m) / (2.0 * internal::deltaH))).toImplementation();
    } else {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) = r_p.boxMinus(r_m) / (2.0 * internal::deltaH);
    }
  }

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateTranslationalJacobianWorldToBodyComInWorldFrame(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  typename std::remove_const<decltype(state)>::type stateTemp;

  for (int i=0; i<state.getNumberOfGeneralizedVelocities(); ++i) {
    // Positive perturbation to i-th entry of the configuration vector.
    stateTemp = state.boxPlus(internal::deltaH, i);
    robotModel.setState(stateTemp, true, true);

    // Get the position of the body when state has been perturbated with +delta.
    Eigen::Vector3d p_p;
    robotModel.getPositionWorldToBodyCom(p_p, branch, node, RobotModelType_::CoordinateFrameEnum::WORLD);

    // Negative perturbation to i-th entry of the configuration vector.
    stateTemp = state.boxPlus(-internal::deltaH, i);
    robotModel.setState(stateTemp, true, true);

    // Get the position of the body when state has been perturbated with -delta.
    Eigen::Vector3d p_m;
    robotModel.getPositionWorldToBodyCom(p_m, branch, node, RobotModelType_::CoordinateFrameEnum::WORLD);

    // Compute the contribution to the jacobian through finite differences.
    G.block<3,1>(0, i) =  (p_p - p_m) / (2.0 * internal::deltaH);
  }

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateRotationalJacobianWorldToBody(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  typename std::remove_const<decltype(state)>::type stateTemp;

  for (int i=0; i<state.getNumberOfGeneralizedVelocities(); ++i) {
    // Positive perturbation to i-th entry of the configuration vector.
    stateTemp = state.boxPlus(internal::deltaH, i);
    robotModel.setState(stateTemp, true, true);

    const romo::RotationQuaternion r_p =
        romo::RotationQuaternion(romo::RotationMatrix(
            robotModel.getOrientationWorldToBody(branch, node).transpose()));

    // Negative perturbation to i-th entry of the configuration vector.
    stateTemp = state.boxPlus(-internal::deltaH, i);
    robotModel.setState(stateTemp, true, true);

    const romo::RotationQuaternion r_m =
        romo::RotationQuaternion(romo::RotationMatrix(
            robotModel.getOrientationWorldToBody(branch, node).transpose()));

    // Compute the contribution to the jacobian through finite differences.
    G.block<3,1>(0, i) =  r_p.boxMinus(r_m) / (2.0 * internal::deltaH);

    if (frame == RobotModelType_::CoordinateFrameEnum::BASE) {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) = state.getOrientationBaseToWorld().inverseRotate(
          romo::Vector(r_p.boxMinus(r_m) / (2.0 * internal::deltaH))
      ).toImplementation();
    } else {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) =  r_p.boxMinus(r_m) / (2.0 * internal::deltaH);
    }
  }

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateTranslationalJacobianWorldToWholeBodyCom(
    RobotModelType_& robotModel,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();

  for (auto i=0; i<state.getNumberOfGeneralizedVelocities(); ++i) {
    // Positive perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(internal::deltaH, i), true, true);
    const Eigen::Vector3d p_p = robotModel.getPositionWorldToCom(RobotModelType_::CoordinateFrameEnum::WORLD);

    // Negative perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(-internal::deltaH, i), true, true);
    const Eigen::Vector3d p_m = robotModel.getPositionWorldToCom(RobotModelType_::CoordinateFrameEnum::WORLD);

    if (frame == RobotModelType_::CoordinateFrameEnum::BASE) {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) = state.getOrientationBaseToWorld().inverseRotate(
          romo::Vector((p_p - p_m) / (2.0 * internal::deltaH))
      ).toImplementation();
    } else {
      // Compute the contribution to the jacobian through finite differences.
      G.block<3,1>(0, i) =  (p_p - p_m) / (2.0 * internal::deltaH);
    }
  }

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateTranslationalJacobianTimeDerivativeWorldToWholeBodyComTimeBased(
    RobotModelType_& robotModel,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  typename std::remove_const<decltype(state)>::type stateCopy;

  constexpr auto nU = state.getNumberOfGeneralizedVelocities();
  const Eigen::VectorXd qDt_plus = state.getGeneralizedVelocities()*internal::deltaT;

  stateCopy = state;
  for (int i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) {
    stateCopy = stateCopy.boxPlus(qDt_plus(i), i);
  }
  robotModel.setState(stateCopy, true, true);
  Eigen::MatrixXd dj_p = Eigen::MatrixXd::Zero(3, nU);
  robotModel.getJacobianTranslationWorldToCom(dj_p, frame);

  stateCopy = state;
  for (int i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) {
    stateCopy = stateCopy.boxPlus(-qDt_plus(i), i);
  }
  robotModel.setState(stateCopy, true, true);
  Eigen::MatrixXd dj_m = Eigen::MatrixXd::Zero(3, nU);
  robotModel.getJacobianTranslationWorldToCom(dj_m, frame);

  G = (dj_p - dj_m) / (2.0 * internal::deltaT);

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateTranslationalJacobianTimeDerivativeWorldToPointOnBodyInWorldFrame(
    RobotModelType_& robotModel,
    const Eigen::Vector3d positonBodyToPointOnBodyInBodyFrame,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  constexpr auto nU = state.getNumberOfGeneralizedVelocities();
  G = Eigen::MatrixXd::Zero(3, nU);

  for (auto i=0; i<nU; ++i) {
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(3, nU);
    estimateTranslationHessianWorldToPointOnBodyForState(robotModel, positonBodyToPointOnBodyInBodyFrame, branch, node, RobotModelType_::CoordinateFrameEnum::WORLD, i, hessian);
    G += hessian*state.getGeneralizedVelocities()(i);
  }
}

template<typename RobotModelType_>
static void estimateTranslationalJacobianTimeDerivativeWorldToBodyInWorldFrame(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    Eigen::MatrixXd& G) {
  estimateTranslationalJacobianTimeDerivativeWorldToPointOnBodyInWorldFrame(robotModel, Eigen::Vector3d::Zero(), branch, node, G);
}

template<typename RobotModelType_>
static void estimateTranslationalJacobianTimeDerivativeWorldToPointOnBodyTimeBased(
    RobotModelType_& robotModel,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  typename std::remove_const<decltype(state)>::type stateCopy;

  constexpr auto nU = state.getNumberOfGeneralizedVelocities();
  const Eigen::VectorXd qDt_plus = state.getGeneralizedVelocities()*internal::deltaT;

  stateCopy = state;
  for (auto i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) { stateCopy = stateCopy.boxPlus(qDt_plus(i), i); }
  robotModel.setState(stateCopy, true, true);
  Eigen::MatrixXd dj_p = Eigen::MatrixXd::Zero(3, nU);
  robotModel.getJacobianTranslationWorldToPointOnBody(dj_p, positionBodyToPointOnBodyInBodyFrame, branch, node, frame);

  stateCopy = state;
  for (auto i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) { stateCopy = stateCopy.boxPlus(-qDt_plus(i), i); }
  robotModel.setState(stateCopy, true, true);
  Eigen::MatrixXd dj_m = Eigen::MatrixXd::Zero(3, nU);
  robotModel.getJacobianTranslationWorldToPointOnBody(dj_m, positionBodyToPointOnBodyInBodyFrame, branch, node, frame);

  G = (dj_p - dj_m) / (2.0 * internal::deltaT);

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateTranslationalJacobianTimeDerivativeWorldToBodyTimeBased(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::MatrixXd& G) {
  estimateTranslationalJacobianTimeDerivativeWorldToPointOnBodyTimeBased(robotModel, Eigen::Vector3d::Zero(), branch, node, frame, G);
}

template<typename RobotModelType_>
static void estimateRotationalJacobianTimeDerivativeWorldToBodyTimeBased(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  typename std::remove_const<decltype(state)>::type stateCopy;

  constexpr auto nU = state.getNumberOfGeneralizedVelocities();
  const Eigen::VectorXd qDt_plus = state.getGeneralizedVelocities()*internal::deltaT;

  stateCopy = state;
  for (int i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) { stateCopy = stateCopy.boxPlus(qDt_plus(i), i); }
  robotModel.setState(stateCopy, true, true);
  Eigen::MatrixXd dj_p = Eigen::MatrixXd::Zero(3, nU);
  robotModel.getJacobianRotationWorldToBody(dj_p, branch, node, frame);

  stateCopy = state;
  for (int i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) { stateCopy = stateCopy.boxPlus(-qDt_plus(i), i); }
  robotModel.setState(stateCopy, true, true);
  Eigen::MatrixXd dj_m = Eigen::MatrixXd::Zero(3, nU);
  robotModel.getJacobianRotationWorldToBody(dj_m, branch, node, frame);

  G = (dj_p - dj_m) / (2.0 * internal::deltaT);

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateSpatialJacobianTimeDerivativeWorldToBodyTimeBased(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    typename RobotModelType_::CoordinateFrameEnum frame,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  typename std::remove_const<decltype(state)>::type stateCopy;

  constexpr auto nU = state.getNumberOfGeneralizedVelocities();
  const Eigen::VectorXd qDt_plus = state.getGeneralizedVelocities()*internal::deltaT;

  stateCopy = state;
  for (int i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) { stateCopy = stateCopy.boxPlus(qDt_plus(i), i); }
  robotModel.setState(stateCopy, true, true);
  Eigen::MatrixXd dj_p = Eigen::MatrixXd::Zero(6, nU);
  robotModel.getJacobianSpatialWorldToBody(dj_p, branch, node, frame);

  stateCopy = state;
  for (int i=0; i<stateCopy.getNumberOfGeneralizedVelocities(); ++i) { stateCopy = stateCopy.boxPlus(-qDt_plus(i), i); }
  robotModel.setState(stateCopy, true, true);
  Eigen::MatrixXd dj_m = Eigen::MatrixXd::Zero(6, nU);
  robotModel.getJacobianSpatialWorldToBody(dj_m, branch, node, frame);

  G = (dj_p - dj_m) / (2.0 * internal::deltaT);

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateRotationalJacobianTimeDerivativeWorldToBodyInWorldFrame(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  constexpr auto nU = state.getNumberOfGeneralizedVelocities();
  G = Eigen::MatrixXd::Zero(3, nU);

  for (auto i=0; i<nU; ++i) {
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(3, nU);
    estimateRotationHessianWorldToBodyForState(robotModel, branch, node, RobotModelType_::CoordinateFrameEnum::WORLD, i, hessian);
    G += hessian*state.getGeneralizedVelocities()(i);
  }
}

template<typename RobotModelType_>
static void estimateSpatialJacobianTimeDerivativeWorldToBodyInWorldFrame(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    Eigen::MatrixXd& G) {
  const auto state = robotModel.getState();
  constexpr auto nU = state.getNumberOfGeneralizedVelocities();
  G = Eigen::MatrixXd::Zero(6, nU);

  for (auto i=0; i<nU; ++i) {
    Eigen::MatrixXd hessianRotational = Eigen::MatrixXd::Zero(3, nU);
    estimateRotationHessianWorldToBodyForState(robotModel, branch, node, RobotModelType_::CoordinateFrameEnum::WORLD, i, hessianRotational);

    Eigen::MatrixXd hessianTranslational = Eigen::MatrixXd::Zero(3, nU);
    estimateTranslationHessianWorldToBodyForState(robotModel, branch, node, RobotModelType_::CoordinateFrameEnum::WORLD, i, hessianTranslational);

    Eigen::MatrixXd hessianSpatial = Eigen::MatrixXd::Zero(6, nU);
    hessianSpatial.topRows(3) = hessianRotational;
    hessianSpatial.bottomRows(3) = hessianTranslational;

    G += hessianSpatial*state.getGeneralizedVelocities()(i);
  }
}

template<typename RobotModelType_>
static void estimateMassMatrix(RobotModelType_& robotModel, Eigen::MatrixXd& massMatrix) {
  massMatrix = Eigen::MatrixXd::Zero(robotModel.getDofCount(), robotModel.getDofCount());

  for(const auto& body: robotModel.getBodyContainer()) {
    if (body->getIsFixedBody()) {
      continue;
    }
    // Translational kinetic energy contribution
    Eigen::MatrixXd jacobianTranslationWorldToBodyComInWorldFrame =
        Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
    estimateTranslationalJacobianWorldToBodyComInWorldFrame(
        robotModel, body->getBranchEnum(), body->getBodyNodeEnum(), jacobianTranslationWorldToBodyComInWorldFrame);
    massMatrix += body->getMass()*jacobianTranslationWorldToBodyComInWorldFrame.transpose()*jacobianTranslationWorldToBodyComInWorldFrame;

    // Rotational kinetic energy contribution
    Eigen::MatrixXd jacobianRotationWorldToBodyInWorldFrame =
        Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
    estimateRotationalJacobianWorldToBody(
        robotModel, body->getBranchEnum(), body->getBodyNodeEnum(), RobotModelType_::CoordinateFrameEnum::WORLD, jacobianRotationWorldToBodyInWorldFrame);

    const auto& rotationWorldToBody = body->getOrientationWorldToBody();
    const auto& inertiaTensorBody = body->getInertiaMatrix();
    massMatrix += jacobianRotationWorldToBodyInWorldFrame.transpose()*rotationWorldToBody.transpose()
                * inertiaTensorBody*rotationWorldToBody*jacobianRotationWorldToBodyInWorldFrame;
  }
}

template<typename RobotModelType_>
static void estimateGravityTerms(RobotModelType_& robotModel, Eigen::VectorXd& gravityTerms) {
  gravityTerms = Eigen::VectorXd::Zero(robotModel.getDofCount());

  const Eigen::Vector3d& gravityVectorInWorldFrame = robotModel.getGravityVectorInWorldFrame();

  for(const auto& body: robotModel.getBodyContainer()) {
    if (body->getIsFixedBody()) {
      continue;
    }

    Eigen::MatrixXd jacobianTranslationWorldToBodyComInWorldFrame = Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
    romo::finite_differences::estimateTranslationalJacobianWorldToBodyComInWorldFrame(
        robotModel, body->getBranchEnum(), body->getBodyNodeEnum(), jacobianTranslationWorldToBodyComInWorldFrame);
    gravityTerms += -body->getMass()*jacobianTranslationWorldToBodyComInWorldFrame.transpose()*gravityVectorInWorldFrame;
  }
}

template<typename RobotModelType_>
static void estimateChristoffelSymbol(
    RobotModelType_& robotModel, unsigned int i, unsigned int j, unsigned int k, double& symbol) {
  constexpr auto nU = RobotModelType_::RobotState::getNumberOfGeneralizedVelocities();

  Eigen::MatrixXd Bp = Eigen::MatrixXd::Zero(nU, nU);
  Eigen::MatrixXd Bm = Eigen::MatrixXd::Zero(nU, nU);

  const auto state = robotModel.getState();

  // Positive perturbation to k-th entry of the configuration vector.
  robotModel.setState(state.boxPlus(internal::deltaH, k), true, true);
  Bp.setZero();
  robotModel.getMassInertiaMatrix(Bp);

  // Negative perturbation to k-th entry of the configuration vector.
  robotModel.setState(state.boxPlus(-internal::deltaH, k), true, true);
  Bm.setZero();
  robotModel.getMassInertiaMatrix(Bm);
  const double dbij_dqk = (Bp(i,j)-Bm(i,j))/(2.0 * internal::deltaH);


  // Positive perturbation to j-th entry of the configuration vector.
  robotModel.setState(state.boxPlus(internal::deltaH, j), true, true);
  Bp.setZero();
  robotModel.getMassInertiaMatrix(Bp);

  // Negative perturbation to j-th entry of the configuration vector.
  robotModel.setState(state.boxPlus(-internal::deltaH, j), true, true);
  Bm.setZero();
  robotModel.getMassInertiaMatrix(Bm);
  const double dbik_dqj = (Bp(i,k)-Bm(i,k))/(2.0 * internal::deltaH);


  // Positive perturbation to i-th entry of the configuration vector.
  robotModel.setState(state.boxPlus(internal::deltaH, i), true, true);
  Bp.setZero();
  robotModel.getMassInertiaMatrix(Bp);

  // Negative perturbation to i-th entry of the configuration vector.
  robotModel.setState(state.boxPlus(-internal::deltaH, i), true, true);
  Bm.setZero();
  robotModel.getMassInertiaMatrix(Bm);
  const double dbjk_dqi = (Bp(j,k)-Bm(j,k))/(2.0 * internal::deltaH);

  symbol = 0.5*(dbij_dqk + dbik_dqj - dbjk_dqi);

  // Set back the original state
  robotModel.setState(state, true, true);
}

template<typename RobotModelType_>
static void estimateCoriolisAndCentrifugalTermsFromChristoffelTerms(RobotModelType_& robotModel, Eigen::VectorXd& coriolisAndCentrifugalTerms) {
  constexpr auto nU = RobotModelType_::RobotState::getNumberOfGeneralizedVelocities();
  const Eigen::VectorXd generalizedVelocitiesCopy = robotModel.getState().getGeneralizedVelocities();

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(nU, nU);
  std::vector<Eigen::MatrixXd> christoffelSymbolsTensor(nU, Eigen::MatrixXd::Zero(nU, nU));

  for (auto i=0; i<nU; ++i) {
    for (auto j=0; j<nU; ++j) {
      for (auto k=0; k<nU; ++k) {
        double cijk = 0.0;

        if (k<j) {
          cijk = christoffelSymbolsTensor[i](k,j);
          christoffelSymbolsTensor[i](j,k) = cijk;
        } else {
          estimateChristoffelSymbol(robotModel, i, j, k, cijk);
          christoffelSymbolsTensor[i](j,k) = cijk;
        }

        C(i,j) += cijk*generalizedVelocitiesCopy(k);
      }
    }
  }

  coriolisAndCentrifugalTerms = C*generalizedVelocitiesCopy;
}

template<typename RobotModelType_>
static void estimateCoriolisAndCentrifugalTerms(RobotModelType_& robotModel, Eigen::VectorXd& coriolisAndCentrifugalTerms) {
  constexpr auto nU = RobotModelType_::RobotState::getNumberOfGeneralizedVelocities();
  const auto generalizedVelocitiesCopy = robotModel.getState().getGeneralizedVelocities();

  coriolisAndCentrifugalTerms = Eigen::VectorXd::Zero(robotModel.getDofCount());

  for(const auto& body: robotModel.getBodyContainer()) {
    if (body->getIsFixedBody()) {
      continue;
    }

    const auto branch = body->getBranchEnum();
    const auto node = body->getBodyNodeEnum();

    Eigen::MatrixXd jacobianTranslationWorldToBodyCom =
        Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
    robotModel.getJacobianTranslationWorldToBodyCom(
        jacobianTranslationWorldToBodyCom, branch, node,
        RobotModelType_::CoordinateFrameEnum::WORLD);

    Eigen::MatrixXd jacobianRotationWorldToBody =
        Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
    robotModel.getJacobianRotationWorldToBody(
        jacobianRotationWorldToBody, branch, node,
        RobotModelType_::CoordinateFrameEnum::WORLD);

    Eigen::MatrixXd jacobianTranslationTimeDerivativeWorldToBodyCom =
        Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
    robotModel.getJacobianTranslationTimeDerivativeWorldToPointOnBody(
        jacobianTranslationTimeDerivativeWorldToBodyCom, body->getPositionBodyToBodyCom(RobotModelType_::CoordinateFrameEnum::BODY), branch, node,
        RobotModelType_::CoordinateFrameEnum::WORLD);

    Eigen::MatrixXd jacobianRotationTimeDerivativeWorldToBody =
        Eigen::MatrixXd::Zero(3, robotModel.getDofCount());
    robotModel.getJacobianRotationTimeDerivativeWorldToBody(
        jacobianRotationTimeDerivativeWorldToBody, branch, node,
        RobotModelType_::CoordinateFrameEnum::WORLD);

    const double bodyMass = body->getMass();
    const auto& rotationWorldToBody = body->getOrientationWorldToBody();
    const auto& inertiaTensorBodyInBodyFrame = body->getInertiaMatrix();
    const auto inertiaTensorBodyInWorldFrame =
        rotationWorldToBody.transpose()*inertiaTensorBodyInBodyFrame*rotationWorldToBody;

    const Eigen::Vector3d angularVelocityBody =
        robotModel.getAngularVelocityWorldToBody(branch, node, RobotModelType_::CoordinateFrameEnum::WORLD);

    coriolisAndCentrifugalTerms +=
        bodyMass*jacobianTranslationWorldToBodyCom.transpose()*jacobianTranslationTimeDerivativeWorldToBodyCom*generalizedVelocitiesCopy
      + jacobianRotationWorldToBody.transpose()* (
          inertiaTensorBodyInWorldFrame*jacobianRotationTimeDerivativeWorldToBody*generalizedVelocitiesCopy
        + angularVelocityBody.cross( inertiaTensorBodyInWorldFrame*angularVelocityBody)
      );

  }

}

template<typename RobotModelType_>
static void estimateNonlinearEffects(RobotModelType_& robotModel, Eigen::VectorXd& nonlinearEffects) {
  constexpr auto nU = RobotModelType_::RobotState::getNumberOfGeneralizedVelocities();
  const Eigen::VectorXd generalizedVelocitiesCopy = robotModel.getState().getGeneralizedVelocities();

  Eigen::MatrixXd nonlinearEffectsMatrix = Eigen::MatrixXd::Zero(nU, nU);
  std::vector<Eigen::MatrixXd> christoffelSymbolsTensor(nU, Eigen::MatrixXd::Zero(nU, nU));

  for (auto i=0; i<nU; ++i) {
    for (auto j=0; j<nU; ++j) {
      for (auto k=0; k<nU; ++k) {
        double cijk = 0.0;

        if (k<j) {
          cijk = christoffelSymbolsTensor[i](k,j);
          christoffelSymbolsTensor[i](j,k) = cijk;
        } else {
          estimateChristoffelSymbol(robotModel, i, j, k, cijk);
          christoffelSymbolsTensor[i](j,k) = cijk;
        }

        nonlinearEffectsMatrix(i,j) += cijk*generalizedVelocitiesCopy(k);
      }
    }
  }

  Eigen::VectorXd gravityTerms;
  estimateGravityTerms(robotModel, gravityTerms);

  nonlinearEffects = nonlinearEffectsMatrix*generalizedVelocitiesCopy + gravityTerms;
}

template<typename RobotModelType_>
static void estimateManipulabilityGradientWorldToBody(
    RobotModelType_& robotModel,
    typename RobotModelType_::BranchEnum branch,
    typename RobotModelType_::BodyNodeEnum node,
    Eigen::VectorXd& manipulabilityGradient) {
  constexpr auto nU = RobotModelType_::RobotState::getNumberOfGeneralizedVelocities();
  const auto state = robotModel.getState();

  manipulabilityGradient = Eigen::VectorXd::Zero(nU);

  for (auto i=0; i<state.getNumberOfGeneralizedVelocities(); ++i) {
    // Positive perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(internal::deltaH, i), true, true);

    Eigen::MatrixXd j_p = Eigen::MatrixXd::Zero(3, nU);
    robotModel.getJacobianTranslationWorldToBody(
        j_p, branch, node,
        RobotModelType_::CoordinateFrameEnum::WORLD);

    const double m_p = std::sqrt((j_p*j_p.transpose()).determinant());

    // Positive perturbation to i-th entry of the configuration vector.
    robotModel.setState(state.boxPlus(-internal::deltaH, i), true, true);

    Eigen::MatrixXd j_m = Eigen::MatrixXd::Zero(3, nU);
    robotModel.getJacobianTranslationWorldToBody(
        j_m, branch, node,
        RobotModelType_::CoordinateFrameEnum::WORLD);

    const double m_m = std::sqrt((j_m*j_m.transpose()).determinant());

    manipulabilityGradient(i) = (m_p - m_m) / (2.0 * internal::deltaH);
  }
}

} /* namespace romo */
} /* namespace finite_differences */
