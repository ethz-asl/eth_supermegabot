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
* @file     ControllerBase.hpp
* @author   Christian Gehring, Gabriel Hottiger
* @date     Dec, 2014
* @note     Restructured, June 2016
*/

#pragma once

// STL
#include <string>
#include <atomic>
#include <memory>

// Boost
#include <boost/thread/shared_mutex.hpp>

namespace roco {

//! Controller (Adaptee) Implementation
/*! This controller can implement 0-n interfaces using a variadic template.
 *
 */
template<typename State_, typename Command_, class... Interfaces_>
class ControllerBase: public Interfaces_... {
 public:
  //! Typedef of the state of the robot.
  typedef State_ State;
  //! Typedef of the command of the robot.
  typedef Command_ Command;

 public:
  ControllerBase();
  virtual ~ControllerBase();

  /*! @returns the name of the controller
   */
  virtual const std::string& getName() const;

  /*! Sets the name of the controller.
   * @param name
   */
  virtual void setName(const std::string& name);

  /*! @returns the parameter path
   */
  virtual const std::string& getParameterPath() const;

  /*! Sets the parameter path
   * @param path
   */
  virtual void setParameterPath(const std::string& path);

  //! @returns true if the controller is initialized.
  bool isInitialized() const;

  //! @returns true if the controller has been created.
  bool isCreated() const;

  //! @returns true if the controller is running.
  bool isRunning() const;

  /*! Set state and command of the controller with according mutexes
   * @param state         the state of the robot.
   * @param mutexState    mutex of the robot state
   * @param command       the command container
   * @param mutexCommand  mutex of the command container
   * This method should be implemented by the adapter.
   */
  virtual void setStateAndCommand(std::shared_ptr<State> state,
                                  std::shared_ptr<boost::shared_mutex> mutexState,
                                  std::shared_ptr<Command> command,
                                  std::shared_ptr<boost::shared_mutex> mutexCommand) = 0;

  /*! @returns the state of the robot.
   * This method should be implemented by the adapter.
   */
  virtual const State& getState() const = 0;

  /*! @returns a mutex to protect access to the state.
   * This method should be implemented by the adapter.
   */
  virtual boost::shared_mutex& getStateMutex() = 0;

  /*! @returns the command.
   * This method should be implemented by the adapter.
   */
  virtual const Command& getCommand() const = 0;

  /*! @returns a mutex to protect access to the command.
   * This method should be implemented by the adapter.
   */
  virtual boost::shared_mutex& getCommandMutex() = 0;

  /*! @returns the command.
   * This method should be implemented by the adapter.
   */
  virtual Command& getCommand() = 0;

 protected:
  //! Name of the controller
  std::string name_;

  //! Path of the parameters
  std::string parameterPath_;

  //! Indicates if the controller is created.
  std::atomic_bool isCreated_;

  //! Indicates if the controller is initialized.
  std::atomic_bool isInitialized_;

  //! Indicates if the controller is running.
  std::atomic_bool isRunning_;

};

} /* namespace roco */

#include "ControllerBase.tpp"
