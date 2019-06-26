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
 * @file     ControllerAdapter.hpp
 * @author   Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */
#pragma once

// Roco
#include "roco/controllers/controllers.hpp"
#include "roco/model/CommandInterface.hpp"
#include "roco/model/StateInterface.hpp"

// Rocoma
#include "rocoma/controllers/ControllerExtensionImplementation.hpp"

// Boost
#include <boost/thread.hpp>

// STL
#include <assert.h>
#include <atomic>
#include <exception>
#include <memory>
#include <type_traits>

namespace rocoma {

template <typename Controller_, typename State_, typename Command_>
class ControllerAdapter : virtual public roco::ControllerAdapterInterface,
                          public ControllerExtensionImplementation<Controller_, State_, Command_> {
 public:
  //! Convenience typedefs
  using Base = ControllerExtensionImplementation<Controller_, State_, Command_>;
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;

 public:
  //! Default constructor
  ControllerAdapter() = default;

  //! Default destructor
  ~ControllerAdapter() override = default;

  //! Implementation of the adapter interface (roco::ControllerAdapterInterface)
  /*! Adapts the adaptees create(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  bool createController(double dt) override;

  /*! Adapts the adaptees initialize(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  bool initializeController(double dt) override;

  /*! Adapts the adaptees advance(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  bool advanceController(double dt) override;

  /*! Adapts the adaptees reset(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  bool resetController(double dt) override;

  /*! Adapts the adaptees prestop(dt) function.
   * @returns true if successful
   */
  bool preStopController() override;

  /*! Adapts the adaptees stop(dt) function.
   * @returns true if successful
   */
  bool stopController() override;

  /*! Adapts the adaptees cleanup() function.
   * @returns true if successful
   */
  bool cleanupController() override;

  /*! Adapts the adaptees swap() function.
   * @param dt  time step [s]
   * @param state  state of the previous controller
   * @returns true if successful
   */
  bool swapController(double dt, const roco::ControllerSwapStateInterfacePtr& swapState) override;

  /*! Use this method to get the state of the controller. Must be thread-safe parallel to advance.
   * @returns state
   */
  bool getControllerSwapState(roco::ControllerSwapStateInterfacePtr& swapState) override;

  /*! Use this method to set a shared module to the controller.
   * @param   module reference to module to be set
   * @returns true if successful
   */
  bool addControllerSharedModule(const roco::SharedModulePtr& module) override;

  /*! Sets if the real robot is controlled or only a simulated version.
   * @param flag indicating robot type
   */
  void setIsRealRobot(bool isRealRobot) override { this->isRealRobot_ = isRealRobot; }

  /*! This function gets the name of the controller.
   * @returns controller name
   */
  const std::string& getControllerName() const override { return this->getName(); }

  /*! This function indicates whether the controller was initialized.
   * @returns true iff controller is initialized
   */
  bool isControllerInitialized() const override { return this->isInitialized(); }

  /*! This function indicates whether the controller is being stopped
   * @returns true iff controller is being stopped
   */
  bool isBeingStopped() const override { return isBeingStopped_; }

  /*! This function sets whether the controller is being stopped
   * @param isBeeingStopped flag indicating whether controller is being stopped
   */
  void setIsBeingStopped(bool isBeingStopped) override { isBeingStopped_ = isBeingStopped; }

  /*! This function indicates whether the controller is running (meaning it is the currently advanced controller)
   * @returns true iff controller is being stopped
   */
  bool isRunning() const override { return this->isRunning_; }

  /*! This function sets whether the controller is running (meaning it is the currently advanced controller)
   * @param isRunning flag indicating whether controller is running
   */
  void setIsRunning(bool isRunning) override { this->isRunning_ = isRunning; }

 protected:
  /*! Update the robot state. (Check for limits)
   * @param dt          time step [s]
   * @param checkState  indicates if state should be checked against its limits
   * @returns true if successful
   */
  bool updateState(double dt, bool checkState = true);

  /*! Update the command. (Limit the references)
   * @param dt          time step [s]
   * @returns true if successful
   */
  bool updateCommand(double dt);

 protected:
  std::atomic_bool isBeingStopped_{false};
};

}  // namespace rocoma

#include <rocoma/controllers/ControllerAdapter.tpp>
