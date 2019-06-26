/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Gabriel Hottiger
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
* @file     EmergencyControllerPlugin.hpp
* @author   Gabriel Hottiger
* @date     Jul, 2016
*/

#pragma once

// rocoma_plugin
#include "rocoma_plugin/interfaces/EmergencyControllerPluginInterface.hpp"

// rocoma
#include "rocoma/controllers/EmergencyControllerAdapter.hpp"

// pluginlib
#include <pluginlib/class_list_macros.h>

/*!
 *   Export your emergency controller as a EmergencyControllerPlugin in order to load it as a plugin.
 *   This macro is a wrapper to PLUGINLIB_EXPORT_CLASS, for templated classes.
 *   Protects typedefs in internal namespace.
 */
#define ROCOMA_EXPORT_EMERGENCY_CONTROLLER(name, state, command, controller)                              \
        namespace emergency_plugin_##name_internal{                                                       \
          using name = rocoma_plugin::EmergencyControllerPlugin<controller , state , command>;            \
          using PluginBase = rocoma_plugin::EmergencyControllerPluginInterface<state , command>;          \
          PLUGINLIB_EXPORT_CLASS(name, PluginBase)                                                        \
        }


namespace rocoma_plugin {

//!  Plugin based emergency controller adapter.
/*!
 *   Export your emergency controller as a EmergencyControllerPlugin in order to load it as a plugin.
 */
template<typename Controller_, typename State_, typename Command_>
class EmergencyControllerPlugin: public rocoma::EmergencyControllerAdapter<Controller_, State_, Command_>,
                                 public rocoma_plugin::EmergencyControllerPluginInterface<State_, Command_>
{

};

} /* namespace rocoma_plugin */
