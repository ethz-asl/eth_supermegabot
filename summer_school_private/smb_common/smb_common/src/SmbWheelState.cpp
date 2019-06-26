




   // DELETE








// /*! 
//  * @file  SeActuatorState.cpp
//  * @author  Christian Gehring
//  * @date  Mar 18, 2015
//  * @version 1.0
//  *
//  */
// // series elastic actuator
// #include "series_elastic_actuator/SeActuatorState.hpp"


// namespace series_elastic_actuator {


// SeActuatorState::SeActuatorState() {}

// SeActuatorState::~SeActuatorState() {}

// const any_measurements::Time& SeActuatorState::getStamp() const {
//   return stamp_;
// }

// void SeActuatorState::setStamp(const any_measurements::Time& stamp) {
//   stamp_ = stamp;
// }

// const uint32_t& SeActuatorState::getStatusword() const {
//   return statusword_;
// }

// void SeActuatorState::setStatusword(uint32_t statusword) {
//   statusword_ = statusword;
// }

// const double& SeActuatorState::getCurrent() const {
//   return current_;
// }

// void SeActuatorState::setCurrent(double current) {
//   current_ = current;
// }

// const double& SeActuatorState::getGearPosition() const {
//   return gearPosition_;
// }

// void SeActuatorState::setGearPosition(double gearPosition) {
//   gearPosition_ = gearPosition;
// }

// const double& SeActuatorState::getGearVelocity() const {
//   return gearVelocity_;
// }

// void SeActuatorState::setGearVelocity(double gearVelocity) {
//   gearVelocity_ = gearVelocity;
// }

// const double& SeActuatorState::getJointPosition() const {
//   return jointPosition_;
// }

// void SeActuatorState::setJointPosition(double jointPosition) {
//   jointPosition_ = jointPosition;
// }

// const double& SeActuatorState::getJointVelocity() const {
//   return jointVelocity_;
// }

// void SeActuatorState::setJointVelocity(double jointVelocity) {
//   jointVelocity_ = jointVelocity;
// }

// const double& SeActuatorState::getJointAcceleration() const {
//   return jointAcceleration_;
// }

// void SeActuatorState::setJointAcceleration(double jointAcceleration) {
//   jointAcceleration_ = jointAcceleration;
// }

// const double& SeActuatorState::getJointTorque() const {
//   return jointTorque_;
// }

// void SeActuatorState::setJointTorque(double jointTorque) {
//   jointTorque_ = jointTorque;
// }

// const any_measurements::Imu& SeActuatorState::getImu() const {
//   return imu_;
// }

// void SeActuatorState::setImu(const any_measurements::Imu& imu) {
//   imu_ = imu;
// }

// std::ostream& operator<<(std::ostream& out, const SeActuatorState& state) {
//   out << "Statusword: ";
//   for (int i = 8*sizeof(state.statusword_)-1; i >= 0; i--) {
//     out << ((state.statusword_ & (1 << i)) ? "1" : "0");
//   }
//   out << "Current: " << state.current_ << std::endl;
//   out << "Gear position: " << state.gearPosition_ << std::endl;
//   out << "Gear velocity: " << state.gearVelocity_ << std::endl;
//   out << "Joint position: " << state.jointPosition_ << std::endl;
//   out << "Joint velocity: " << state.jointVelocity_ << std::endl;
//   out << "Joint acceleration: " << state.jointAcceleration_ << std::endl;
//   out << "Joint torque: " << state.jointTorque_ << std::endl;
//   out << "IMU linear acceleration: " << state.imu_.linearAcceleration_ << std::endl;
//   out << "IMU angular velocity: " << state.imu_.angularVelocity_ << std::endl;
//   return out;
// }

// } // series_elastic_actuator

