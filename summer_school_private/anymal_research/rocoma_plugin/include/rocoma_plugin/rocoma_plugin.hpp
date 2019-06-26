/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Christian Gehring, Gabriel Hottiger
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
 *   * Neither the name of ETH Zurich
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
 * @file     rocoma_plugin.hpp
 * @author   Gabriel Hottiger
 * @date     Aug, 2016
 */


#include "rocoma_plugin/interfaces/ControllerPluginInterface.hpp"
#include "rocoma_plugin/interfaces/ControllerRosPluginInterface.hpp"
#include "rocoma_plugin/interfaces/EmergencyControllerPluginInterface.hpp"
#include "rocoma_plugin/interfaces/EmergencyControllerRosPluginInterface.hpp"
#include "rocoma_plugin/interfaces/FailproofControllerPluginInterface.hpp"
#include "rocoma_plugin/interfaces/SharedModulePluginInterface.hpp"
#include "rocoma_plugin/interfaces/SharedModuleRosPluginInterface.hpp"

#include "rocoma_plugin/plugins/ControllerPlugin.hpp"
#include "rocoma_plugin/plugins/ControllerRosPlugin.hpp"
#include "rocoma_plugin/plugins/ControllerTuplePlugin.hpp"
#include "rocoma_plugin/plugins/ControllerTupleRosPlugin.hpp"
#include "rocoma_plugin/plugins/EmergencyControllerPlugin.hpp"
#include "rocoma_plugin/plugins/EmergencyControllerRosPlugin.hpp"
#include "rocoma_plugin/plugins/FailproofControllerPlugin.hpp"
#include "rocoma_plugin/plugins/SharedModulePlugin.hpp"
#include "rocoma_plugin/plugins/SharedModuleRosPlugin.hpp"
