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
 * @file     ControllerExtensionImplementation.tpp
 * @author   Christian Gehring, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

#include <any_worker/Worker.hpp>
#include <rocoma/common/WorkerWrapper.hpp>
#include <message_logger/message_logger.hpp>

namespace rocoma {

template<typename Controller_, typename State_, typename Command_>
roco::WorkerHandle ControllerExtensionImplementation<Controller_, State_, Command_>::addWorker(const roco::WorkerOptions&  options) {
  std::unique_lock<std::mutex> lockWorkerManager(mutexWorkerManager_);
  WorkerWrapper wrapper(options);
  workerManager_.addWorker(options.name_, 1.0/options.frequency_,
                           std::bind(&WorkerWrapper::workerCallback, wrapper, std::placeholders::_1), options.priority_, options.autostart_);

  MELO_INFO_STREAM("[Rocoma][" << this->getName() << "] Add worker " << options.name_ << "!");
  return roco::WorkerHandle(options.name_);
}

template<typename Controller_, typename State_, typename Command_>
roco::WorkerHandle ControllerExtensionImplementation<Controller_, State_, Command_>::addWorker(roco::Worker& worker) {
  std::unique_lock<std::mutex> lockWorkerManager(mutexWorkerManager_);
  WorkerWrapper wrapper(worker.options_);
  workerManager_.addWorker(worker.options_.name_, 1.0/worker.options_.frequency_,
                           std::bind(&WorkerWrapper::workerCallback, wrapper, std::placeholders::_1), worker.options_.priority_, worker.options_.autostart_);
  worker.workerStartCallback_ = boost::bind(&ControllerExtensionImplementation<Controller_,State_, Command_>::startWorker, this, _1);
  worker.workerCancelCallback_ = boost::bind(&ControllerExtensionImplementation<Controller_,State_, Command_>::cancelWorker, this, _1, _2 );
  worker.handle_.name_ = worker.options_.name_;
  MELO_INFO_STREAM("[Rocoma][" << this->getName() << "] Add worker " << worker.handle_.name_ << "!");
  return worker.handle_;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerExtensionImplementation<Controller_, State_, Command_>::startWorker(const roco::WorkerHandle& workerHandle) {
  std::unique_lock<std::mutex> lockWorkerManager(mutexWorkerManager_);
  MELO_INFO_STREAM("[Rocoma][" << this->getName() << "] Start worker " << workerHandle.name_ << "!");
  workerManager_.startWorker(workerHandle.name_);
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerExtensionImplementation<Controller_, State_, Command_>::stopWorker(const roco::WorkerHandle& workerHandle, bool block) {
  std::unique_lock<std::mutex> lockWorkerManager(mutexWorkerManager_);
  MELO_INFO_STREAM("[Rocoma][" << this->getName() << "] Stop worker " << workerHandle.name_ << "!");
  workerManager_.stopWorker(workerHandle.name_, block);
  return true;
}

template<typename Controller_, typename State_, typename Command_>
bool ControllerExtensionImplementation<Controller_, State_, Command_>::cancelWorker(const roco::WorkerHandle& workerHandle, bool block) {
  std::unique_lock<std::mutex> lockWorkerManager(mutexWorkerManager_);
  MELO_INFO_STREAM("[Rocoma][" << this->getName() << "] Cancel worker " << workerHandle.name_ << "!");
  workerManager_.cancelWorker(workerHandle.name_, block);
  return true;
}

} /** namespace rocoma */
