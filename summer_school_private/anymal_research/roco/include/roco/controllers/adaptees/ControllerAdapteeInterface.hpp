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
 * @file     ControllerAdapteeInterface.hpp
 * @author   Christian Gehring, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

#pragma once

// roco
#include "roco/controllers/ControllerSwapStateInterface.hpp"
#include "roco/controllers/SharedModule.hpp"

namespace roco {

//!   Abstract interface class for controller adaptees.
/*!
 *   This interface is used in the controller implementation.
 */
class ControllerAdapteeInterface
{
 public:
  //! Empty constructor
  ControllerAdapteeInterface() { }

  //! Empty destructor
  virtual ~ControllerAdapteeInterface() { }

 protected:
  /*! Use this method instead of the constructor to create objects.
   * This method is only called once during the whole lifetime of the controller.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool create(double dt) = 0;

  /*! This initializes the controller before the advance method is called.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool initialize(double dt) = 0;

  /*! This advances the controller.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool advance(double dt) = 0;

  /*! This resets the controller assuming it was already initialized once.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool reset(double dt) = 0;

  /*! This prepares the controller for a stop/switch. Unregister, delete everything that
   *  is not essential to advance (e.g. ros communication)
   * @returns true if successful
   */
  virtual bool preStop() = 0;

  /*! This stops the controller. Kill everything that is not used when controller is not
   *  advancing anymore. (E.g running threads etc.)
   * @returns true if successful
   */
  virtual bool stop() = 0;

  /*! Use this method instead of the destructor to destroy objects.
   * This method is only called once at the end of the lifetime of the controller.
   * @returns true if successful
   */
  virtual bool cleanup() = 0;

  /*! Use this method to swap from another controller.
   * @param dt  time step [s]
   * @param state  State received from the previous controller
   * @returns true if successful
   */
  virtual bool swap(double dt, const ControllerSwapStateInterfacePtr& swapState) = 0;

  /*! Use this method to get the state of the controller. Must be thread-safe parallel to advance.
   * @param   swapState reference to state to be set
   * @returns true if successful
   */
  virtual bool getSwapState(ControllerSwapStateInterfacePtr& swapState) = 0;

  /*! Use this method to set a shared module to the controller.
   * @param   module reference to module to be set
   * @returns true if successful
   */
  virtual bool addSharedModule(const SharedModulePtr& module) = 0;
};

} /* namespace roco */
