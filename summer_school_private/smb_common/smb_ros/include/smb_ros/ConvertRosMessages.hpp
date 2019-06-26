/*!
 * @file    ConvertRosMessages.hpp
 * @author  Koen Kraemer
 * @date    Aug 23, 2018
 *
 */
#pragma once


// smb_commmon
#include <smb_common/SmbCommands.hpp>
#include <smb_common/SmbReadings.hpp>

#include <any_measurements_ros/any_measurements_ros.hpp>

// smb_msgs
#include <smb_msgs/SmbCommands.h>
#include <smb_msgs/SmbReadings.h>

#include <smb_description/SmbDescription.hpp>

#include <array>
#include <std_utils/std_utils.hpp>
#include <smb_model/SmbModel.hpp>

namespace smb_ros {

template<typename ConcreteDescription_>
class ConvertRosMessages
{
  ConvertRosMessages() = delete;
  virtual ~ConvertRosMessages() = default;

public:
  using RD = ConcreteDescription_;
  using RT = typename ConcreteDescription_::ConcreteTopology;
  using SmbActuatorEnum = typename RT::SmbActuatorEnum;

  static void writeToMessage(smb_msgs::SmbCommands& message, const smb_common::SmbCommands<ConcreteDescription_>& commands);
  static void readFromMessage(smb_common::SmbCommands<ConcreteDescription_>& command, const smb_msgs::SmbCommands& message);

  static void writeToMessage(smb_msgs::SmbReadings& message, const smb_common::SmbReadings<ConcreteDescription_>& measurements);
  static void readFromMessage(smb_common::SmbReadings<ConcreteDescription_>& measurements, const smb_msgs::SmbReadings& message);
};

template<typename Msg_, typename MsgRos_>
class ConversionTraits;

template<>
class ConversionTraits<smb_common::SmbReadings<smb_model::SmbModel::SmbDescription>, smb_msgs::SmbReadings> {
public:
  using SD = smb_model::SmbModel::SmbDescription;
  using Msg = smb_common::SmbReadings<smb_model::SmbModel::SmbDescription>;
  using MsgRos = smb_msgs::SmbReadings;

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    ConvertRosMessages<SD>::writeToMessage(rosMsg, msg);
    return rosMsg;
  }

  inline static Msg convert(const MsgRos& rosMsg) {
    Msg shmMsg;
    ConvertRosMessages<SD>::readFromMessage(shmMsg, rosMsg);
    return shmMsg;
  }

};


template<>
class ConversionTraits<smb_common::SmbCommands<smb_model::SmbModel::SmbDescription>, smb_msgs::SmbCommands> {
public:
  using SD = smb_model::SmbModel::SmbDescription;
  using Msg = smb_common::SmbCommands<smb_model::SmbModel::SmbDescription>;
  using MsgRos = smb_msgs::SmbCommands;

    inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    ConvertRosMessages<SD>::writeToMessage(rosMsg, msg);
    return rosMsg;
  }

  inline static Msg convert(const MsgRos& rosMsg) {
    Msg shmMsg;
    ConvertRosMessages<SD>::readFromMessage(shmMsg, rosMsg);
    return shmMsg;
  }

};

} // smb_ros

#include <smb_ros/ConvertRosMessages.tpp>
