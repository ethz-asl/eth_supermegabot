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
 * @file     EmergencyControllerAdapter.hpp
 * @author   Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

#pragma once

// Roco
#include "roco/controllers/adaptees/EmergencyControllerAdapteeInterface.hpp"
#include "roco/controllers/adapters/EmergencyControllerAdapterInterface.hpp"

// Rocoma
#include "rocoma/controllers/ControllerAdapter.hpp"

// Message logger
#include <message_logger/message_logger.hpp>

// STL
#include <type_traits>
#include <assert.h>
#include <memory>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class EmergencyControllerAdapter: virtual public roco::EmergencyControllerAdapterInterface, public ControllerAdapter<Controller_, State_, Command_>
{
  //! Check if Controller_ template parameter inherits from roco::EmergencyControllerAdapteeInterface
  static_assert(std::is_base_of<roco::EmergencyControllerAdapteeInterface, Controller_>::value,
                "[EmergencyControllerAdapter]: The Controller class does not implement the EmergencyControllerAdatpeeInterface.");

 public:
  //! Convenience typedefs
  using Base = ControllerAdapter<Controller_, State_, Command_>;
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;

 public:
  //! Default constructor
  EmergencyControllerAdapter() { }

  //! Default destructor
  virtual ~EmergencyControllerAdapter() { }

  /*! Adapts the adaptees initializeFast(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool initializeControllerFast(double dt) {

    // Check if the controller was created.
    if (!this->isCreated()) {
      MELO_WARN_STREAM("[Rocoma][" << this->getName() << "] Was not created!");
      return false;
    }

    // Initialize the controller now.
#ifdef NDEBUG
    try
#endif
    {
      // Update the state.
      if(!this->updateState(dt, false)) {
        return false;
      }

      // Initialize controller
      if (!this->initializeFast(dt)) {
        MELO_WARN_STREAM("[Rocoma][" << this->getName() << "] Could not be fast initialized!");
        return false;
      }

      // Update command
      if(!this->updateCommand(dt)) {
        return false;
      }

      this->isInitialized_ = true;

    }
#ifdef NDEBUG
    catch (std::exception& e) {
      MELO_WARN_STREAM("[Rocoma][" << this->getName() << "] Exception caught while fast initializing:\n" << e.what());
      this->isInitialized_ = false;
      return false;
    }
    catch (...) {
      MELO_WARN_STREAM("[Rocoma][" << this->getName() << "] Exception caught while fast initializing!\n");
      this->isInitialized_ = false;
      return false;
    }
#endif

    // Start logging
    this->isRunning_ = true;
    MELO_INFO_STREAM("[Rocoma][" << this->getName() << "] Fast initialized successfully!");

    return true;
  }

};

} // namespace rocoma
