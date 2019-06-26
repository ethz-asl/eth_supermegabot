/*!
* @file 	  smb_failproof_controller_plugin.cpp
* @author   Johannes Pankert
* @date		  09/04/2019
* @version 	1.0
* @brief    Plugin export for controller SmbFailproofController.
*/

// state and command
#include "smb_roco/RocoState.hpp"
#include "smb_roco/RocoCommand.hpp"

// smb_failproof_controller
#include "smb_failproof_controller/SmbFailproofController.hpp"

// rocoma_plugin
#include "rocoma_plugin/rocoma_plugin.hpp"

// export controller plugin
ROCOMA_EXPORT_FAILPROOF_CONTROLLER(
  SmbFailproofController,
  smb_roco::RocoState,
  smb_roco::RocoCommand,
  smb_failproof_controller::SmbFailproofController
)
