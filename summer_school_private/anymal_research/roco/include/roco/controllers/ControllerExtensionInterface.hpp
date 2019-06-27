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
 * @file     ControllerExtensionInterface.hpp
 * @author   Christian Gehring, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

#pragma once

// ROCO
#include "roco/time/Time.hpp"
#include "roco/workers/WorkerOptions.hpp"
#include "roco/workers/WorkerHandle.hpp"
#include "roco/workers/Worker.hpp"

namespace roco {

//!  Abstract interface class for extending controller functionality.
/*!
 * This methods should be implemented by the adapter.
 */
class ControllerExtensionInterface
{
 public:
  //! Empty constructor
  ControllerExtensionInterface() {};

  //! Empty destructor
  virtual ~ControllerExtensionInterface() {};

  /*! Indicates if the real robot is controller or only a simulated version.
   * @returns true if real robot
   */
  virtual bool isRealRobot() const = 0;

  /*! Gets the controller time
   * @returns controller time
   */
  virtual const time::Time& getTime() const = 0;

  /*! Sets the controller time
   * @param time  controller time
   */
  virtual void setTime(const time::Time& time) = 0;

  /*! Indicates if command is checked against its limits
   * @returns true iff command is checked
   */
  virtual bool isCheckingCommand() const = 0;

  /*! Set if command is checked against its limits
   * @param isChecking  flag indicating whether command should be checked against limits
   */
  virtual void setIsCheckingCommand(bool isChecking) = 0;

  /*! Indicates if state is checked against its limits
   * @returns true iff state is checked
   */
  virtual bool isCheckingState() const = 0;

  /*! Set if state is checked against its limits
   * @param isChecking  flag indicating whether state should be checked against limits
   */
  virtual void setIsCheckingState(bool isChecking) = 0;

  /*! Add a worker to the worker queue via options
   * @param options  worker options (priority, timestep, ...)
   * @returns worker handle for the added worker
   */
  virtual roco::WorkerHandle addWorker(const roco::WorkerOptions& options) = 0;

  /*! Add a worker to the worker queue
   * @param worker  worker to be added
   * @returns worker handle for the added worker
   */
  virtual roco::WorkerHandle addWorker(roco::Worker& worker) = 0;

  /*! Start a given worker
   * @param workerHandle  handle to the worker to be started
   * @returns true iff was started successfully
   */
  virtual bool startWorker(const roco::WorkerHandle& workerHandle) = 0;

  /*! Stop a given worker
   * @param workerHandle  handle to the worker to be stopped
   * @param block  flag indicating whether this should block until thread can be joined (default = false)
   * @returns true iff was stopped successfully
   */
  virtual bool stopWorker(const roco::WorkerHandle& workerHandle, bool block=false) = 0;

  /*! Cancel a given worker
   * @param workerHandle  handle to the worker to be cancelled
   * @param block  flag indicating whether this should block until thread can be joined (default = false)
   * @returns true iff was cancelled successfully
   */
  virtual bool cancelWorker(const roco::WorkerHandle& workerHandle, bool block=false) = 0;

};

} /* namespace roco */
