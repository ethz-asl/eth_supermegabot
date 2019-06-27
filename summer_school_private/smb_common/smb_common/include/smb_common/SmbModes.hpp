//
// Created by Koen Kramer on 23.08.18.
//
#pragma once

#include <ros/types.h>

namespace smb_common {

  typedef int16_t SmbModeType;
  enum SmbMode : SmbModeType {
//      DISCONNECTED                             = 0,  // No communication
      FREEZE = 0,                                      // Freeze
//      CHANGE_COMMAND_TYPE,                           // Change the wheel command mode
      MODE_WHEEL_VELOCITY,                            // Track wheel velocity
      MODE_WHEEL_TORQUE,                              // Track wheel torque
      WHEEL_DC_CMD                                   // Command motor duty cycle open loop
  };

} //namespace smb_common
