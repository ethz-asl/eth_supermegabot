/*!
 * @file     ExtendedRobotState.tpp
 * @author   Dominic Jud
 * @date     Dez 8, 2016
 */

#include <romo/ExtendedRobotState.hpp>

namespace romo {

template<typename ConcreteDescription_>
const typename ExtendedRobotState<ConcreteDescription_>::JointAccelerations& ExtendedRobotState<ConcreteDescription_>::getJointAccelerations() const {
  return jointAccelerations_;
}

template<typename ConcreteDescription_>
typename ExtendedRobotState<ConcreteDescription_>::JointAccelerations& ExtendedRobotState<ConcreteDescription_>::getJointAccelerations() {
  return jointAccelerations_;
}

template<typename ConcreteDescription_>
const typename ExtendedRobotState<ConcreteDescription_>::JointTorques& ExtendedRobotState<ConcreteDescription_>::getJointTorques() const {
  return jointTorques_;
}

template<typename ConcreteDescription_>
typename ExtendedRobotState<ConcreteDescription_>::JointTorques& ExtendedRobotState<ConcreteDescription_>::getJointTorques() {
  return jointTorques_;
}

template<typename ConcreteDescription_>
const LinearAcceleration& ExtendedRobotState<ConcreteDescription_>::getLinearAccelerationBaseInWorldFrame() const {
	return linearAccelerationBaseInWorldFrame_;
}

template<typename ConcreteDescription_>
LinearAcceleration& ExtendedRobotState<ConcreteDescription_>::getLinearAccelerationBaseInWorldFrame() {
	return linearAccelerationBaseInWorldFrame_;
}

template<typename ConcreteDescription_>
const AngularAcceleration& ExtendedRobotState<ConcreteDescription_>::getAngularAccelerationBaseInBaseFrame() const {
	return angularAccelerationBaseInBaseFrame_;
}

template<typename ConcreteDescription_>
AngularAcceleration& ExtendedRobotState<ConcreteDescription_>::getAngularAccelerationBaseInBaseFrame() {
	return angularAccelerationBaseInBaseFrame_;
}

template<typename ConcreteDescription_>
void ExtendedRobotState<ConcreteDescription_>::setJointAccelerations(const JointAccelerations& jointAccelerations) {
  jointAccelerations_ = jointAccelerations;
}

template<typename ConcreteDescription_>
void ExtendedRobotState<ConcreteDescription_>::setJointTorques(const JointTorques& jointTorques) {
	jointTorques_ = jointTorques;
}

template<typename ConcreteDescription_>
void ExtendedRobotState<ConcreteDescription_>::setLinearAccelerationBaseInWorldFrame(const LinearAcceleration& linearAccelerationBaseInWorldFrame)
{
	linearAccelerationBaseInWorldFrame_ = linearAccelerationBaseInWorldFrame;
}

template<typename ConcreteDescription_>
void ExtendedRobotState<ConcreteDescription_>::setAngularAccelerationBaseInBaseFrame(const AngularAcceleration& angularAccelerationBaseInBaseFrame)
{
	angularAccelerationBaseInBaseFrame_ = angularAccelerationBaseInBaseFrame;
}

template<typename ConcreteDescription_>
void ExtendedRobotState<ConcreteDescription_>::setRandom() {
  RobotState<ConcreteDescription_>::setRandom();
  jointAccelerations_.setRandom();
  jointTorques_.setRandom();
  linearAccelerationBaseInWorldFrame_.setRandom();
  angularAccelerationBaseInBaseFrame_.setRandom();
}

template<typename ConcreteDescription_>
void ExtendedRobotState<ConcreteDescription_>::setZero() {
  RobotState<ConcreteDescription_>::setZero();
  jointAccelerations_ = JointAccelerations::Zero();
  jointTorques_ = JointTorques::Zero();
  linearAccelerationBaseInWorldFrame_ = LinearAcceleration::Zero();
  angularAccelerationBaseInBaseFrame_ = AngularAcceleration::Zero();
}

}