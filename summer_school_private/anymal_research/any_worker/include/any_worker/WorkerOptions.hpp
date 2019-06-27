/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Philipp Leemann
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
 *   * Neither the name of Robotic Systems Lab nor ETH Zurich
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
 * @file	WorkerOptions.hpp
 * @author	Philipp Leemann
 * @date	Jul 8, 2016
 */

#pragma once

#include <atomic>
#include <string>

#include "any_worker/RateOptions.hpp"
#include "any_worker/WorkerEvent.hpp"

namespace any_worker {

using WorkerCallback = std::function<bool(const WorkerEvent&)>;
using WorkerCallbackFailureReaction = std::function<void(void)>;

struct WorkerOptions : public RateOptions {
    WorkerOptions():
        RateOptions(),
        callback_(),
        callbackFailureReaction_([](){}),
        defaultPriority_(0),
        destructWhenDone_(false)
    {
    }

    WorkerOptions(const std::string& name, const double timestep, const WorkerCallback& callback, const int priority=0):
        RateOptions(name, timestep),
        callback_(callback),
        callbackFailureReaction_([](){}),
        defaultPriority_(priority),
        destructWhenDone_(false)
    {

    }

    WorkerOptions(const std::string& name,
                  const double timestep,
                  const WorkerCallback& callback,
                  const WorkerCallbackFailureReaction& callbackFailureReaction,
                  const int priority=0):
      RateOptions(name, timestep),
      callback_(callback),
      callbackFailureReaction_(callbackFailureReaction),
      defaultPriority_(priority),
      destructWhenDone_(false)
    {

    }

    WorkerOptions(const WorkerOptions& other):
        RateOptions(other),
        callback_(other.callback_),
        callbackFailureReaction_(other.callbackFailureReaction_),
        defaultPriority_(other.defaultPriority_),
        destructWhenDone_(other.destructWhenDone_)
    {

    }

    WorkerOptions(WorkerOptions&& other):
      RateOptions(std::move(other)),
      callback_(std::move(other.callback_)),
      callbackFailureReaction_(std::move(other.callbackFailureReaction_)),
      defaultPriority_(other.defaultPriority_),
      destructWhenDone_(other.destructWhenDone_)
    {

    }

    /*!
     * The primary worker callback to be called
     */
    WorkerCallback callback_;

    /*!
     * The reaction callback to be called when the primary indicates error (returns false)
     */
    WorkerCallbackFailureReaction callbackFailureReaction_;

    /*!
     * priority of the underlying thread, integer between 0 and 99 with 0 beeing the lowest priority.
     */
    int defaultPriority_;

    /*!
     * if set to true and timestep=0 (run callback only once), the worker will be destructed by the WorkerManager
     */
    bool destructWhenDone_;

};

} // namespace any_worker
