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
 * @file     ControllerAdapter.tpp
 * @author   Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

// Message logger
#include <message_logger/message_logger.hpp>

// Signal logger
#include <signal_logger/signal_logger.hpp>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::createController(double dt)
{
  if (this->isCreated()) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Has already been created!");
    return true;
  }

#ifdef NDEBUG
  try
#endif
  {
    // Create controller
    if (!this->create(dt)) {
      this->isCreated_ = false;
      MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Could not be created!");
      return false;
    }

    // Set flag
    this->isCreated_ = true;

  }
#ifdef NDEBUG
  catch (std::exception& e) {
    //! return false (let manager handle this)
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while creating: " << e.what());
    this->isCreated_ = false;
    return false;
  }
  catch (...) {
    //! return false (let manager handle this)
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while creating!");
    this->isCreated_ = false;
    return false;
  }
#endif

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::initializeController(double dt)
{
  // Check if the controller was created.
  if (!this->isCreated()) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Not created on initialize!");
    return false;
  }

  // Reset instead of initialization if the controller has been already initialized.
  if (this->isInitialized()) {
    return resetController(dt);
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
    if (!this->initialize(dt)) {
      MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Could not be initialized!");
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
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while initializing:\n" << e.what());
    this->isInitialized_ = false;
    return false;
  }
  catch (...) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while initializing!\n");
    this->isInitialized_ = false;
    return false;
  }
#endif

  // Set flags
  MELO_DEBUG_STREAM("[Rocoma][" << this->getControllerName() << "] Successfully initialized!");

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::advanceController(double dt)
{
  // Check if controller is initialized
  if (!this->isInitialized()) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Not initialized on advance!");
    return false;
  }

#ifdef NDEBUG
  try
#endif
  {
    // Advance controller
    if(!this->updateState(dt)) {
      return false;
    }

    if (!this->advance(dt)) {
      MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Could not advance!");
      return false;
    }

    // Update commands
    if(!this->updateCommand(dt)) {
      return false;
    }

  }
#ifdef NDEBUG
  catch (std::exception& e) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while advancing: " << e.what());
    return false;
  }
  catch (...) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while advancing! ");
    return false;
  }
#endif
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::resetController(double dt)
{
  // Check if controller was created
  if (!this->isCreated()) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Has not been created!");
    return false;
  }

  // Initialize if first call
  if (!this->isInitialized()) {
    return initializeController(dt);
  }

#ifdef NDEBUG
  try
#endif
  {
    // Update state
    if(!updateState(dt, false) ) {
      return false;
    }

    // Reset controller
    if (!this->reset(dt)) {
      MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Could not reset controller!");
      return false;
    }

    // Update command
    if(!updateCommand(dt)) {
      return false;
    }

  }
#ifdef NDEBUG
  catch (std::exception& e) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while resetting: " << e.what());
    return false;
  }
  catch (...) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while resetting!");
    return false;
  }
#endif

  MELO_DEBUG_STREAM("[Rocoma][" << this->getControllerName() << "] Reset successfully!");

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::cleanupController()
{
  MELO_INFO_STREAM("[Rocoma][" << this->getControllerName() << "] Clean up!");

  // Check if controller was created
  if (!this->isCreated()) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Was not created!");
    return false;
  }

  // Cleanup controllers
#ifdef NDEBUG
  try
#endif
  {
    if (!this->cleanup()) {
      MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Could not clean up!");
      return false;
    }

  }
#ifdef NDEBUG
  catch (std::exception& e) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while cleaning up: " << e.what());
    return false;
  }
  catch (...) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while cleaning up!");
    return false;
  }
#endif

  // Set flags
  this->isInitialized_ = false;
  this->isCreated_ = false;

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::stopController()
{
  // Stop controller
#ifdef NDEBUG
  try
#endif
  {
    if(!this->stop()) {
      MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Could not be stopped!");
      return false;
    }
  }
#ifdef NDEBUG
  catch (std::exception& e) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while stopping: " << e.what());
    return false;
  }
  catch (...) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while stopping!");
    return false;
  }
#endif

  return true;
}


template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::preStopController()
{

#ifdef NDEBUG
  try
#endif
  {
    if(!this->preStop()) {
      MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Could not prepare to stop controller!");
      return false;
    }
  }
#ifdef NDEBUG
  catch (std::exception& e) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while pre-stopping: " << e.what());
    return false;
  }
  catch (...) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while pre-stopping! ");
    return false;
  }
#endif

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_, State_, Command_>::swapController(double dt, const roco::ControllerSwapStateInterfacePtr& swapState)
{
  // Check if the controller was created.
  if (!this->isCreated()) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Not created on swap!");
    return false;
  }

  // todo: If not initialized -> initialize? -> then also set flags below

  // Initialize the controller now.
#ifdef NDEBUG
  try
#endif
  {
    // Update the state.
    if(!this->updateState(dt, false)) {
      return false;
    }

    // Swap controller
    if (!this->swap(dt, swapState)) {
      MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Could not be swapped!");
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
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while swapping:\n" << e.what());
    this->isInitialized_ = false;
    return false;
  }
  catch (...) {
    MELO_WARN_STREAM("[Rocoma][" << this->getControllerName() << "] Exception caught while swapping!\n");
    this->isInitialized_ = false;
    return false;
  }
#endif

  MELO_DEBUG_STREAM("[Rocoma][" << this->getControllerName() << "] Successfully swapped!");

  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::getControllerSwapState(roco::ControllerSwapStateInterfacePtr& swapState)
{
  return this->getSwapState(swapState);
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::addControllerSharedModule(const roco::SharedModulePtr& module)
{
  return this->addSharedModule(module);
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::updateState(double dt, bool checkState)
{
  this->time_.setNow();

  if (checkState && this->isCheckingState_) {
    boost::shared_lock<boost::shared_mutex> lock(this->getStateMutex());
    if (!this->getState().checkState()) {
      MELO_ERROR_STREAM("[Rocoma][" << this->getControllerName() << "] Bad state!");
      return false;
    }
  }
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerAdapter<Controller_,State_, Command_>::updateCommand(double dt)
{
  if (this->isCheckingCommand_) {
    boost::unique_lock<boost::shared_mutex> lock(this->getCommandMutex());
    if (!this->getCommand().limitCommand()) {
      MELO_ERROR_STREAM("[Rocoma][" << this->getControllerName() << "] The command is invalid!");
      return false;
    }
  }

  return true;
}

}  // namespace rocoma