/*!
 * @file    smb_commands.hpp
 * @author  Koen Kraemer
 * @date    Aug 21, 2018
 *
 */


#pragma once

#include <smb_msgs/SmbCommands.h>

#include <smb_common/SmbModes.hpp>

#include <smb_description/SmbDescription.hpp>

namespace smb_description_ros {

static void initializeCommands(smb_msgs::SmbCommands& commands) {
  using SmbActuatorEnum = typename smb_description::SmbTopology::SmbActuatorEnum;

  constexpr auto numActuators = static_cast<unsigned int>(SmbActuatorEnum::SIZE);

  // Actuator commands
  commands.wheelCommands.clear();
  commands.wheelCommands.resize(numActuators);

  for (const auto &key : smb_description::SmbDescription::getKeys<SmbActuatorEnum>()) {
    const auto actuatorId = key.getId();
    commands.wheelCommands[actuatorId].header.frame_id = key.getName();
    commands.wheelCommands[actuatorId].mode = smb_common::SmbMode::FREEZE;
  }
}
}
