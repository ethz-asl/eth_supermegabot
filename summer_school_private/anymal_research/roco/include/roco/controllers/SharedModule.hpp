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
 * @file     SharedModuleInterface.hpp
 * @author   Gabriel Hottiger
 * @date     Aug, 2017
 */

#pragma once

#include <memory>
#include <mutex>

namespace roco {

//!   Interface class for shared modules.
/*!
 *   This interface can be used to share a module between different roco controllers. The implementer is responsible
 *   for thread safety.
 */
class SharedModule
{
 public:
  //! Empty constructor
  SharedModule() = default;

  //! Empty destructor
  virtual ~SharedModule() = default;

  /*! Use this method instead of the constructor to create objects.
   * This method is only called once during the whole lifetime of the shared module.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool create(double dt) { return true; }

  //! @return name of the module
  const std::string & getName() const { return name_; }

  //! @param name name of the module
  void setName(const std::string & name) { name_ = name; }

  //! @returns the parameter path
  const std::string& getParameterPath() const { return parameterPath_; }

  /*! Sets the parameter path
   * @param parameterPath
   */
  void setParameterPath(const std::string& parameterPath) { parameterPath_ = parameterPath; }

  //! @return least granular lock
  std::mutex& acquireMutex() const { return smMutex_; }

 protected:
  //! Name of the shared module
  std::string name_;
  //! Path of the parameters
  std::string parameterPath_;
  //! Mutex to protect the shared module (least granular lock)
  mutable std::mutex smMutex_;

};

using SharedModulePtr = std::shared_ptr<SharedModule>;


} /* namespace roco */
