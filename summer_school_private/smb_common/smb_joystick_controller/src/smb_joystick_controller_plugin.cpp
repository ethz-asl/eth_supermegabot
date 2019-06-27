/*!
* @file 	  smb_joystick_controller_plugin.cpp
* @author   Johannes Pankert
* @date		  27/11/2018
* @version 	1.0
* @brief    Plugin export for controller SmbJoystickController.
*/

// state and command
#include "smb_roco/RocoState.hpp"
#include "smb_roco/RocoCommand.hpp"

// smb_joystick_controller
#include "smb_joystick_controller/SmbJoystickController.hpp"

// rocoma_plugin
#include "rocoma_plugin/rocoma_plugin.hpp"

// export controller plugin
ROCOMA_EXPORT_CONTROLLER_ROS(
  SmbJoystickControllerPlugin,
  smb_roco::RocoState,
  smb_roco::RocoCommand,
  smb_joystick_controller::SmbJoystickController
);
