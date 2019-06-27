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
 * @file     ControllerExtensionImplementation.hpp
 * @author   Christian Gehring, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

#pragma once

// rocoma
#include "rocoma/controllers/ControllerImplementation.hpp"

// roco
#include "roco/time/TimeStd.hpp"
#include "roco/workers/Worker.hpp"
#include "roco/workers/WorkerHandle.hpp"
#include "roco/workers/WorkerOptions.hpp"

// any worker
#include "any_worker/WorkerManager.hpp"

// STL
#include <memory>
#include <atomic>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class ControllerExtensionImplementation: public ControllerImplementation<Controller_, State_, Command_> {

  //! Check if Controller_ template parameter inherits from roco::Controller<State_, Command_>
  static_assert(std::is_base_of<roco::Controller<State_, Command_>, Controller_>::value,
                "[ControllerExtensionImplementation]: The Controller class does not inherit from roco::Controller<State_, Command_>!" );

 public:
  //! Convenience typedefs
  using Base = ControllerImplementation<Controller_, State_, Command_>;
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;

 public:
  //! Default Constructor
  ControllerExtensionImplementation():
    Base(),
    isRealRobot_(true),
    isCheckingCommand_(true),
    isCheckingState_(true),
    time_(),
    workerManager_()
  {

  }

  //! Default Destructor
  virtual ~ControllerExtensionImplementation() { }

  /*! Indicates if the real robot is controller or only a simulated version.
   * @returns true if real robot
   */
  virtual bool isRealRobot() const                      { return isRealRobot_; }

  /*! Gets the controller time
   * @returns controller time
   */
  virtual const roco::time::Time& getTime() const       { return static_cast<const roco::time::Time&>(time_); }

  /*! Sets the controller time
   * @param time  controller time
   */
  virtual void setTime(const roco::time::Time& time)    { time_ = time;}

  /*! Indicates if command is checked against its limits
   * @returns true iff command is checked
   */
  virtual bool isCheckingCommand() const                { return isCheckingCommand_; }

  /*! Set if command is checked against its limits
   * @param isChecking  flag indicating whether command should be checked against limits
   */
  virtual void setIsCheckingCommand(bool isChecking)    { isCheckingCommand_ = isChecking; }

  /*! Indicates if state is checked against its limits
   * @returns true iff state is checked
   */
  virtual bool isCheckingState() const                  { return isCheckingState_; }

  /*! Set if state is checked against its limits
   * @param isChecking  flag indicating whether state should be checked against limits
   */
  virtual void setIsCheckingState(bool isChecking)      { isCheckingState_ = isChecking; }

  /*! Add a worker to the worker queue via options
   * @param options  worker options (priority, timestep, ...)
   * @returns worker handle for the added worker
   */
  virtual roco::WorkerHandle addWorker(const roco::WorkerOptions& options);

  /*! Add a worker to the worker queue
   * @param worker  worker to be added
   * @returns worker handle for the added worker
   */
  virtual roco::WorkerHandle addWorker(roco::Worker& worker);

  /*! Start a given worker
   * @param workerHandle  handle to the worker to be started
   * @returns true iff was started successfully
   */
  virtual bool startWorker(const roco::WorkerHandle& workerHandle);

  /*! Stop a given worker
   * @param workerHandle  handle to the worker to be stopped
   * @param block  flag indicating whether this should block until thread can be joined (default = false)
   * @returns true iff was stopped successfully
   */
  virtual bool stopWorker(const roco::WorkerHandle& workerHandle, bool block = false);

  /*! Cancel a given worker
   * @param workerHandle  handle to the worker to be cancelled
   * @param block  flag indicating whether this should block until thread can be joined (default = false)
   * @returns true iff was cancelled successfully
   */
  virtual bool cancelWorker(const roco::WorkerHandle& workerHandle, bool block = false);

 protected:
  //! Indicates if the real robot is controller or only a simulated version.
  std::atomic<bool> isRealRobot_;
  //! Indicates if command is checked for limits
  std::atomic<bool> isCheckingCommand_;
  //! Indicates if state is checked for validity
  std::atomic<bool> isCheckingState_;
  //! Time
  roco::time::TimeStd time_;
  //! Worker Manager
  any_worker::WorkerManager workerManager_;
  //! Worker Manager Mutex
  std::mutex mutexWorkerManager_;

};

} /** namespace rocoma */

#include <rocoma/controllers/ControllerExtensionImplementation.tpp>
