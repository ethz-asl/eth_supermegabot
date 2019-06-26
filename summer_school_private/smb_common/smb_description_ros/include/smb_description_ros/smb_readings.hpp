/*!
 * @file    smb_readings.hpp
 * @author  Koen Kraemer
 * @date    Aug 21, 2018
 *
 */

#pragma once

#include <smb_msgs/SmbReadings.h>

#include <smb_common/SmbModes.hpp>

#include <smb_description/SmbDescription.hpp>

namespace smb_description_ros {

static void initializeReadings(smb_msgs::SmbReadings &readings) {
  using SmbActuatorEnum = typename smb_description::SmbTopology::SmbActuatorEnum;

  constexpr auto numActuators = static_cast<unsigned int>(SmbActuatorEnum::SIZE);

  // Actuator readings
  readings.wheelReadings.clear();
  readings.wheelReadings.resize(numActuators);

  for (const auto &key : smb_description::SmbDescription::getKeys<SmbActuatorEnum>()) {
    const auto actuatorId = key.getId();
    readings.wheelReadings[actuatorId].header.frame_id = key.getName();
//    readings.wheelReadings[actuatorId].mode = smb_common::SmbMode::MODE_VELOCITY;
  }
}
}
