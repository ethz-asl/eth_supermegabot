/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Dario Bellicoso
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
 * @file     WorkerWrapper.hpp
 * @author   Dario Bellicoso, Gabriel Hottiger
 * @date     Jun 18, 2015
 * @note     Restructured Aug 08, 2016
 */

#pragma once

// roco
#include "roco/workers/WorkerOptions.hpp"
#include "roco/workers/WorkerEventInterface.hpp"
#include "roco/workers/WorkerEventStd.hpp"

// any_worker
#include "any_worker/Worker.hpp"
#include "any_worker/WorkerEvent.hpp"

namespace rocoma {

//!  Wrapper for roco worker events using the any_worker package.
class WrapperWorkerEvent : public roco::WorkerEventStd, public any_worker::WorkerEvent {
 public:
  //! Delete default constructor
  WrapperWorkerEvent() = delete;

  /*! Create a wrapped worker event from an any_woker event
   * @param event  any_worker event
   * @returns wrapped worker event
   */
  WrapperWorkerEvent(const any_worker::WorkerEvent& event ):
    any_worker::WorkerEvent(event)
  {

  }

  //! Default destructor
  virtual ~WrapperWorkerEvent()
  {

  }

};

//!  Wrapper for roco workers using the any_worker package.
class WorkerWrapper
{
 public:
  //! Delete default constructor
  WorkerWrapper() = delete;

  /*! Create a wrapped worker from roco worker options
   * @param options  roco worker options
   * @returns wrapped worker
   */
  WorkerWrapper(roco::WorkerOptions options):
    options_(options)
  {

  }

  //! Default destructor
  virtual ~WorkerWrapper()
  {

  }

  /*! Wrap the worker callback function
   * @param workerEvent  any worker event
   * @returns true if successful
   */
  inline bool workerCallback(const any_worker::WorkerEvent& workerEvent)
  {
    WrapperWorkerEvent event(workerEvent);
    return options_.callback_(event);
  }

 private:
  // Worker options
  roco::WorkerOptions options_;
};

}
