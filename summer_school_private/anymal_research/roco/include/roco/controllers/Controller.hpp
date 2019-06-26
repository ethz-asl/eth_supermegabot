/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Christian Gehring
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
* @file     Controller.hpp
* @author   Christian Gehring, Gabriel Hottiger
* @date     Dec, 2014
* @note     Restructured, June 2016
*/

#pragma once

// Roco
#include "roco/controllers/ControllerBase.hpp"
#include "roco/controllers/ControllerExtensionInterface.hpp"
#include "roco/controllers/adaptees/ControllerAdapteeInterface.hpp"

namespace roco {

//! Controller
/*! Derive this class and implement your own controller.
 *
 */
  template<typename State_, typename Command_>
  class Controller: virtual public ControllerBase<State_, Command_, ControllerAdapteeInterface, ControllerExtensionInterface> {
   public:
    Controller() { }
    virtual ~Controller() { }

    /*! Use this method to swap from another controller.
     * Default: initialize or reset
     * @param dt  time step [s]
     * @param state  State received from the previous controller
     * @returns true if successful
     */
    virtual bool swap(double dt, const ControllerSwapStateInterfacePtr& swapState) {
      return this->isInitialized() ? this->reset(dt) : this->initialize(dt);
    }

    /*! Use this method to get the state of the controller. Must be thread-safe parallel to advance.
     * Default: sets nullptr
     * @param   swapState reference to state to be set
     * @returns true if successful
     */
    virtual bool getSwapState(ControllerSwapStateInterfacePtr& swapState) {
      swapState.reset( nullptr );
      return true;
    }

    /*! Use this method to set a shared module to the controller.
     * Default: do nothing
     * @param   module reference to module to be set
     * @return  true if successful (default: false)
     */
    virtual bool addSharedModule(const SharedModulePtr& module) {
      return false;
    }

  };
}
