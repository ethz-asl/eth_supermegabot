/*!
 * @file     RobotState.tpp
 * @author   Dario Bellicoso
 * @date     Oct, 2016
 */

#include <romo/RobotState.hpp>
#include <ctime>

namespace romo {

template <typename ConcreteDescription_>
void RobotState<ConcreteDescription_>::setPositionWorldToBaseInWorldFrame( const Position& positionWorldToBaseInWorldFrame) {
  positionWorldToBaseInWorldFrame_ = positionWorldToBaseInWorldFrame;
}

template <typename ConcreteDescription_>
void RobotState<ConcreteDescription_>::setOrientationBaseToWorld(const RotationQuaternion& orientationBaseToWorld) {
  orientationBaseToWorld_ = orientationBaseToWorld;
}

template <typename ConcreteDescription_>
typename RobotState<ConcreteDescription_>::GeneralizedCoordinates RobotState<ConcreteDescription_>::getGeneralizedCoordinates() const {
  GeneralizedCoordinates generalizedCoordinates;
  generalizedCoordinates.topRows(3) = this->positionWorldToBaseInWorldFrame_.toImplementation();
  generalizedCoordinates(3) = this->orientationBaseToWorld_.w();
  generalizedCoordinates(4) = this->orientationBaseToWorld_.x();
  generalizedCoordinates(5) = this->orientationBaseToWorld_.y();
  generalizedCoordinates(6) = this->orientationBaseToWorld_.z();
  generalizedCoordinates.bottomRows(RD::getJointsDimension()) = this->jointPositions_.toImplementation();
  return generalizedCoordinates;
}

template <typename ConcreteDescription_>
typename RobotState<ConcreteDescription_>::GeneralizedVelocities RobotState<ConcreteDescription_>::getGeneralizedVelocities() const {
  GeneralizedVelocities generalizedVelocities;
  generalizedVelocities(0) = this->linearVelocityBaseInWorldFrame_.x();
  generalizedVelocities(1) = this->linearVelocityBaseInWorldFrame_.y();
  generalizedVelocities(2) = this->linearVelocityBaseInWorldFrame_.z();
  generalizedVelocities(3) = this->angularVelocityBaseInBaseFrame_.x();
  generalizedVelocities(4) = this->angularVelocityBaseInBaseFrame_.y();
  generalizedVelocities(5) = this->angularVelocityBaseInBaseFrame_.z();
  generalizedVelocities.bottomRows(RD::getJointsDimension()) = this->jointVelocities_.toImplementation();
  return generalizedVelocities;
}

template <typename ConcreteDescription_>
void RobotState<ConcreteDescription_>::setLinearVelocityBaseInWorldFrame(const LinearVelocity& linearVelocityBaseInWorldFrame) {
  linearVelocityBaseInWorldFrame_ =  linearVelocityBaseInWorldFrame;
}

template <typename ConcreteDescription_>
void RobotState<ConcreteDescription_>::setAngularVelocityBaseInBaseFrame(const LocalAngularVelocity& angularVelocityBaseInBaseFrame) {
  angularVelocityBaseInBaseFrame_ = angularVelocityBaseInBaseFrame;
}

template <typename ConcreteDescription_>
const Position& RobotState<ConcreteDescription_>::getPositionWorldToBaseInWorldFrame() const {
  return positionWorldToBaseInWorldFrame_;
}

template <typename ConcreteDescription_>
const RotationQuaternion& RobotState<ConcreteDescription_>::getOrientationBaseToWorld() const {
  return orientationBaseToWorld_;
}

template <typename ConcreteDescription_>
const LinearVelocity& RobotState<ConcreteDescription_>::getLinearVelocityBaseInWorldFrame() const {
  return linearVelocityBaseInWorldFrame_;
}

template <typename ConcreteDescription_>
const LocalAngularVelocity& RobotState<ConcreteDescription_>::getAngularVelocityBaseInBaseFrame() const {
  return angularVelocityBaseInBaseFrame_;
}

template <typename ConcreteDescription_>
const typename RobotState<ConcreteDescription_>::JointPositions& RobotState<ConcreteDescription_>::getJointPositions() const {
  return jointPositions_;
}

template <typename ConcreteDescription_>
typename RobotState<ConcreteDescription_>::JointPositions& RobotState<ConcreteDescription_>::getJointPositions() {
  return jointPositions_;
}

template <typename ConcreteDescription_>
const typename RobotState<ConcreteDescription_>::JointVelocities& RobotState<ConcreteDescription_>::getJointVelocities() const {
  return jointVelocities_;
}

template <typename ConcreteDescription_>
typename RobotState<ConcreteDescription_>::JointVelocities& RobotState<ConcreteDescription_>::getJointVelocities() {
  return jointVelocities_;
}

template <typename ConcreteDescription_>
void RobotState<ConcreteDescription_>::setJointPositions(const JointPositions& jointPositions) {
  jointPositions_ = jointPositions;
}

template <typename ConcreteDescription_>
void RobotState<ConcreteDescription_>::setJointVelocities(const JointVelocities& jointVelocities) {
  jointVelocities_ = jointVelocities;
}

template <typename ConcreteDescription_>
void RobotState<ConcreteDescription_>::setRandom() {
  std::srand(std::time(0)); //use current time as seed for random generator
  jointPositions_.setRandom();
  jointVelocities_.setRandom();
  positionWorldToBaseInWorldFrame_.setRandom();
  linearVelocityBaseInWorldFrame_.setRandom();
  angularVelocityBaseInBaseFrame_.setRandom();
  orientationBaseToWorld_.setRandom();
}

template <typename ConcreteDescription_>
void RobotState<ConcreteDescription_>::setZero() {
  jointPositions_.setZero();
  jointVelocities_.setZero();
  positionWorldToBaseInWorldFrame_.setZero();
  linearVelocityBaseInWorldFrame_.setZero();
  angularVelocityBaseInBaseFrame_.setZero();
  orientationBaseToWorld_.setIdentity();
}

template <typename ConcreteDescription_>
void RobotState<ConcreteDescription_>::setZeroVelocities() {
  jointVelocities_.setZero();
  linearVelocityBaseInWorldFrame_.setZero();
  angularVelocityBaseInBaseFrame_.setZero();
}

}