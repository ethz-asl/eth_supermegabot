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
 * @file	Worker.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#include <functional>
#include <string>
#include <atomic>
#include <thread>

#include "any_worker/Rate.hpp"
#include "any_worker/WorkerEvent.hpp"
#include "any_worker/WorkerOptions.hpp"

namespace any_worker {

class Worker {
public:
    Worker() = delete;

    /*!
     * @param name      name of the worker
     * @param timestep  with a timestep of 0, the callback will be executed as fast as possible, with std::numeric_limits<double>::infinity() only once.
     * @param callback  std::function pointing to the callback
     */
    Worker(const std::string& name, const double timestep, const WorkerCallback& callback);
    Worker(const std::string& name, const double timestep, const WorkerCallback& callback,
           const WorkerCallbackFailureReaction& callbackFailureReaction);
    Worker(const WorkerOptions& options);
    Worker(const Worker&) = delete; // atomics and threads are non-copyable
    Worker(Worker&&); // declare custom move constructor to move atomics

    virtual ~Worker();

    bool start(const int priority=0);
    void stop(const bool wait=true);

    void setTimestep(const double timeStep);
    void setEnforceRate(const bool enforceRate);

    const std::string& getName() const { return options_.name_; }
    const Rate& getRate() const { return rate_; }
    Rate& getRate() { return rate_; }

    bool isRunning() const { return running_; }

    /*!
     * @return true if underlying thread has terminated and deleteWhenDone_ option is set.
     */
    bool isDestructible() const { return done_.load() && options_.destructWhenDone_; }

private:
    void run();

private:
    WorkerOptions options_;

    std::atomic<bool> running_;
    std::atomic<bool> done_;

    std::thread thread_;
    Rate rate_;
};

} // namespace any_worker
