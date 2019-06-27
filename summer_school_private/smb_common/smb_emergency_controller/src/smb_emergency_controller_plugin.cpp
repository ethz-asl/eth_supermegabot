/*!
* @file 	  smb_emergency_controller_plugin.cpp
* @author   Johannes Pankert
* @date		  10/04/2019
* @version 	1.0
* @brief    Plugin export for controller SmbEmergencyController.
*/

// state and command
#include "smb_roco/RocoState.hpp"
#include "smb_roco/RocoCommand.hpp"

// smb_emergency_controller
#include "smb_emergency_controller/SmbEmergencyController.hpp"

// rocoma_plugin
#include "rocoma_plugin/rocoma_plugin.hpp"

// export controller plugin
ROCOMA_EXPORT_EMERGENCY_CONTROLLER(
  SmbEmergencyControllerPlugin,
  smb_roco::RocoState,
  smb_roco::RocoCommand,
  smb_emergency_controller::SmbEmergencyController
);
