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
* @file     FailproofControllerAdapter.hpp
* @author   Christian Gehring, Gabriel Hottiger
* @date     Dec, 2014
* @note     Restructured, June 2016
*/
#pragma once

// Roco
#include "roco/controllers/adapters/FailproofControllerAdapterInterface.hpp"
#include "roco/controllers/FailproofController.hpp"

// Rocoma
#include <rocoma/controllers/ControllerImplementation.hpp>

// STL
#include <type_traits>
#include <assert.h>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
class FailproofControllerAdapter: virtual public roco::FailproofControllerAdapterInterface, public ControllerImplementation<Controller_, State_, Command_>
{
  //! Check if template parameters implement the required interfaces
  static_assert(std::is_base_of<roco::FailproofController<State_, Command_>, Controller_>::value,
                "[FailproofControllerAdapter]: The Controller class does not inherit from roco::FailproofController<State_, Command_>!" );

 public:
  //! Convenience typedefs
  using Controller = Controller_;
  using State = State_;
  using Command = Command_;
  using Base = ControllerImplementation<Controller, State, Command>;


 public:
  //! Default constructor
  FailproofControllerAdapter() { }

  //! Default destructor
  virtual ~FailproofControllerAdapter() { }

  /*! Adapts the adaptees create(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool createController(double dt)
  {
    return this->create(dt);
  }

  /*! Adapts the adaptees advance(dt) function.
   * @param dt  time step [s]
   */
  virtual void advanceController(double dt)
  {
    this->advance(dt);
    return;
  }

  /*! Adapts the adaptees cleanup() function.
   * @returns true if successful
   */
  virtual bool cleanupController()
  {
    return this->cleanup();
  }

  /*! This function gets the name of the controller.
   * @returns controller name
   */
  virtual const std::string& getControllerName() const
  {
    return this->getName();
  }


};

} // namespace rocoma
