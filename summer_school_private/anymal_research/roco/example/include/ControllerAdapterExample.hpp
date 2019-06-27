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
* @file     ControllerAdapterExample.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#pragma once

#include <iostream>
#include <exception>      // std::exception

#include <roco/time/Time.hpp>
#include <roco/time/TimeStd.hpp>

#include <roco/controllers/ControllerAdapterInterface.hpp>
#include <roco/log/log_messages.hpp>

namespace roco {
namespace controllers {


template<typename Controller_>
class ControllerAdapterExample:  public ControllerAdapterInterface, public Controller_
{
 public:
  typedef Controller_ Controller;
  typedef typename Controller::State State;
  typedef typename Controller::Command Command;
 public:
  ControllerAdapterExample(State& state, Command& command) :
    Controller(),
    isRealRobot_(false),
    isCheckingCommand_(true),
    isCheckingState_(true),
    time_(),
    state_(state),
    command_(command)
   {};
  virtual ~ControllerAdapterExample() {};

  virtual bool createController(double dt) {
    if (this->isCreated()) {
      return false;
    }
    try {
      this->isCreated_ = this->create(dt);
    }
    catch (std::exception& e) {
      ROCO_WARN_STREAM("Exception caught: " << e.what());
      this->isCreated_ = false;
    }
    return this->isCreated();
  }

  virtual bool initializeController(double dt) {
    if (!this->isCreated()) {
      return false;
    }
    try {
      this->isInitialized_ = this->initialize(dt);
    }
    catch (std::exception& e) {
      ROCO_WARN_STREAM("Exception caught: " << e.what());
      this->isInitialized_ = false;
    }
    time_.fromSec(0.0);
    state_ = 0.0;
    return this->isInitialized();
  }

  virtual bool advanceController(double dt) {
    if (!this->isInitialized()) {
      return false;
    }
    time_ += dt;
    state_ += 1.0;
    try {
      return this->advance(dt);
    }
    catch (std::exception& e) {
      ROCO_WARN_STREAM("Exception caught: " << e.what());
    }
    return false;
  }

  virtual bool cleanupController() {
    if (!this->isCreated()) {
       return false;
    }
    try {
      return this->cleanup();
    }
    catch (std::exception& e) {
      ROCO_WARN_STREAM("Exception caught: " << e.what());
    }
    return false;
  }

  virtual bool resetController(double dt) {
    if (!this->isCreated()) {
       return false;
    }
    if (!this->isInitialized()) {
       return initializeController(dt);
    }

    try {
      return this->reset(dt);
    }
    catch (std::exception& e) {
      ROCO_WARN_STREAM("Exception caught: " << e.what());
    }
    return false;
  }

  virtual bool changeController() {
    if (!this->isInitialized()) {
       return false;
    }
    try {
      return this->change();
    }
    catch (std::exception& e) {
       ROCO_WARN_STREAM("Exception caught: " << e.what());
    }
    return false;
  }

  virtual const time::Time& getTime() const {
    return static_cast<const time::Time&>(time_);
  }

  virtual void setTime(const time::Time& time) {
     time_ = time;
  }

  virtual bool isCheckingCommand() const {
    return isCheckingCommand_;
  }
  virtual void setIsCheckingCommand(bool isChecking) {
    isCheckingCommand_ = isChecking;
  }

  virtual bool isCheckingState() const {
    return isCheckingState_;
  }

  virtual void setIsCheckingState(bool isChecking) {
    isCheckingState_ = isChecking;
  }

  virtual bool isRealRobot() const {
    return isRealRobot_;
  }
  virtual void setIsRealRobot(bool isRealRobot) {
    isRealRobot_ = isRealRobot;
  }

  virtual const State& getState() const {
    return state_;
  }

  virtual const Command& getCommand() const {
    return command_;
  }

  virtual Command& getCommand() {
    return command_;
  }


 private:
  //! Indicates if the real robot is controller or only a simulated version.
  bool isRealRobot_;
  bool isCheckingCommand_;
  bool isCheckingState_;
  time::TimeStd time_;
  State& state_;
  Command& command_;
};

} /* namespace controllers */
} /* namespace roco */


