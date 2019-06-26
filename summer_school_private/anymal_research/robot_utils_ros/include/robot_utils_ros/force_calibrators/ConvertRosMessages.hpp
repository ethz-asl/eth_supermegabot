#pragma once

#include <std_utils/containers/EnumArray.hpp>

#include <robot_utils/force_calibrators/ForceCalibratorCommand.hpp>

#include <robot_utils_ros/ForceCalibratorCommand.h>
#include <robot_utils_ros/ForceCalibratorCommands.h>

namespace robot_utils_ros {

class ConvertRosMessages
{
public:
  ConvertRosMessages() = delete;
  ~ConvertRosMessages() = default;

  static void writeToMessage(robot_utils_ros::ForceCalibratorCommand& message, const robot_utils::ForceCalibratorCommand& command);
  static void readFromMessage(robot_utils::ForceCalibratorCommand& command, const robot_utils_ros::ForceCalibratorCommand& message);
};

template<typename ConcreteDescription_, typename Msg_, typename MsgRos_>
class ConversionTraits;

//! This conversion is used by cosmo
template<typename ConcreteDescription_>
class ConversionTraits<ConcreteDescription_, std_utils::EnumArray<typename ConcreteDescription_::ContactEnum, robot_utils::ForceCalibratorCommand>, robot_utils_ros::ForceCalibratorCommands> {
 public:
  using Msg = std_utils::EnumArray<typename ConcreteDescription_::ContactEnum, robot_utils::ForceCalibratorCommand>;
  using MsgRos = robot_utils_ros::ForceCalibratorCommands;

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    robot_utils_ros::ForceCalibratorCommand command;
    rosMsg.commands.reserve(ConcreteDescription_::getContactKeys().size());
    for (auto contactKey : ConcreteDescription_::getContactKeys()) {
      ConvertRosMessages::writeToMessage(command, msg[contactKey.getEnum()]);
      rosMsg.commands.push_back(command);
    }
    return rosMsg;
  }

  inline static Msg convert(const MsgRos& rosMsg) {
    Msg msg;
    assert(rosMsg.commands.size() == ConcreteDescription_::getNumLimbs() );
    for (auto contactKey : ConcreteDescription_::getContactKeys()) {
      ConvertRosMessages::readFromMessage(msg[contactKey.getEnum()], rosMsg.commands[contactKey.getId()]);
    }
    return msg;
  }
};

//! This conversion is used by cosmo
template<typename ConcreteDescription_>
class ConversionTraits<ConcreteDescription_, std_utils::EnumArray<typename ConcreteDescription_::ConcreteTopology::ForceSensorEnum, robot_utils::ForceCalibratorCommand>, robot_utils_ros::ForceCalibratorCommands> {
 public:
  using ForceSensorEnum = typename ConcreteDescription_::ConcreteTopology::ForceSensorEnum;
  using Msg = std_utils::EnumArray<ForceSensorEnum, robot_utils::ForceCalibratorCommand>;
  using MsgRos = robot_utils_ros::ForceCalibratorCommands;

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    robot_utils_ros::ForceCalibratorCommand command;
    rosMsg.commands.reserve(ConcreteDescription_:: template getKeys<ForceSensorEnum>().size());
    for (auto forceSensorKey : ConcreteDescription_::template getKeys<ForceSensorEnum>()) {
      ConvertRosMessages::writeToMessage(command, msg[forceSensorKey.getEnum()]);
      rosMsg.commands.push_back(command);
    }
    return rosMsg;
  }

  inline static Msg convert(const MsgRos& rosMsg) {
    Msg msg;
    assert(rosMsg.commands.size() == static_cast<unsigned int>(ForceSensorEnum::SIZE));
    for (auto forceSensorKey : ConcreteDescription_::template getKeys<ForceSensorEnum>()) {
      ConvertRosMessages::readFromMessage(msg[forceSensorKey.getEnum()], rosMsg.commands[forceSensorKey.getId()]);
    }
    return msg;
  }
};


}