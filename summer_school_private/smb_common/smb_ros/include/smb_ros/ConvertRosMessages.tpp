/*!
 * @file    ConvertRosMessages.hpp
 * @author  Markus Staeuble, Koen Kramer
 * @date    Apr 6, 2018
 *
 */

#include <smb_ros/ConvertRosMessages.hpp>

namespace smb_ros {

template<typename ConcreteDescription_>
void ConvertRosMessages<ConcreteDescription_>::writeToMessage(smb_msgs::SmbCommands& message, const smb_common::SmbCommands<RD>& commands) {

  message.wheelCommands.resize(RD::ConcreteDefinitions::getNumWheels());

  for(auto actuatorKey : RD::template getKeys<SmbActuatorEnum>()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();

    message.wheelCommands[actuatorId].header.stamp = any_measurements_ros::toRos(commands.wheelCommands_[actuatorEnum].getStamp());
    message.wheelCommands[actuatorId].mode = commands.wheelCommands_[actuatorEnum].getMode();

    message.wheelCommands[actuatorId].velocity = commands.wheelCommands_[actuatorEnum].getWheelVelocity();
    message.wheelCommands[actuatorId].torque = commands.wheelCommands_[actuatorEnum].getWheelTorque();
  }
}

template<typename ConcreteDescription_>
void ConvertRosMessages<ConcreteDescription_>::readFromMessage(smb_common::SmbCommands<RD>& command, const smb_msgs::SmbCommands& message) {

  assert(message.wheelCommands.size() == RD::ConcreteDefinitions::getNumWheels());

  for(auto actuatorKey : RD::template getKeys<SmbActuatorEnum>()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();

    command.wheelCommands_[actuatorEnum].setStamp(any_measurements_ros::fromRos(message.wheelCommands[actuatorId].header.stamp));
    command.wheelCommands_[actuatorEnum].setMode(message.wheelCommands[actuatorId].mode);

    command.wheelCommands_[actuatorEnum].setWheelVelocity(message.wheelCommands[actuatorId].velocity);
    command.wheelCommands_[actuatorEnum].setWheelTorque(message.wheelCommands[actuatorId].torque);
  }
}


template<typename ConcreteDescription_>
void ConvertRosMessages<ConcreteDescription_>::writeToMessage(smb_msgs::SmbReadings& message, const smb_common::SmbReadings<RD>& measurements){

  message.wheelReadings.resize(RD::ConcreteDefinitions::getNumWheels());

  for(auto actuatorKey : RD::template getKeys<SmbActuatorEnum>()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();

    message.wheelReadings[actuatorId].header.stamp = any_measurements_ros::toRos(measurements.wheelReadings_[actuatorEnum].getStamp());
//    message.wheelReadings[actuatorId].mode = measurements[actuatorEnum].getMode();

    message.wheelReadings[actuatorId].velocity = measurements.wheelReadings_[actuatorEnum].getWheelVelocity();
    message.wheelReadings[actuatorId].torque = measurements.wheelReadings_[actuatorEnum].getWheelTorque();
  }
}


template<typename ConcreteDescription_>
void ConvertRosMessages<ConcreteDescription_>::readFromMessage(smb_common::SmbReadings<RD>& measurements, const smb_msgs::SmbReadings& message) {

  assert(message.wheelReadings.size() == RD::ConcreteDefinitions::getNumWheels());

  for(auto actuatorKey : RD::template getKeys<SmbActuatorEnum>()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();

    measurements.wheelReadings_[actuatorEnum].setStamp(any_measurements_ros::fromRos(message.wheelReadings[actuatorId].header.stamp));
//    measurements.wheelReadings_[actuatorEnum].setMode(message.wheelReadings[actuatorId].mode);

    measurements.wheelReadings_[actuatorEnum].setWheelVelocity(message.wheelReadings[actuatorId].velocity);
    measurements.wheelReadings_[actuatorEnum].setWheelTorque(message.wheelReadings[actuatorId].torque);
  }
}

} // namespace
