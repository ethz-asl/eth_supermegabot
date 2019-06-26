/*!
* @file 	  smb_smb_path_following_controller_plugin.cpp
* @author   todo
* @date		  06/05/2019
* @version 	1.0
* @brief    Plugin export for controller SmbPathFollowingController.
*/

// state and command
#include "smb_roco/RocoState.hpp"
#include "smb_roco/RocoCommand.hpp"

// smb_smb_path_following_controller
#include "smb_path_following_controller/SmbPathFollowingController.hpp"

// rocoma_plugin
#include "rocoma_plugin/rocoma_plugin.hpp"

// export controller plugin
ROCOMA_EXPORT_CONTROLLER_ROS(
  SmbPathFollowingControllerPlugin,
  smb_roco::RocoState,
  smb_roco::RocoCommand,
  smb_path_following::SmbPathFollowingController
);
