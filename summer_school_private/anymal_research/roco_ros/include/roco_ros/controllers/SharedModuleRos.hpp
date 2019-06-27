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
 * @file     SharedModuleRos.hpp
 * @author   Gabriel Hottiger
 * @date     Aug, 2017
 */

#pragma once

#include "roco/controllers/SharedModule.hpp"

#include "ros/ros.h"

#include <memory>

namespace roco_ros {

//!   Interface class for ros shared modules.
/*!
 *   This interface can be used to share a module between different roco controllers. The implementer is responsible
 *   for thread safety.
 */
class SharedModuleRos: virtual public roco::SharedModule
{
 public:
  //! Empty constructor
  SharedModuleRos() = default;

  //! Empty destructor
  virtual ~SharedModuleRos() = default;

  /*! Get the ros node handle associated with this shared module.
   * @returns the ros nodehandle
   */
  ros::NodeHandle getNodeHandle() const
  {
    return nh_;
  }

  /*! Get reference to the ros node handle associated with this shared module.
   * @returns the ros nodehandle
   */
  ros::NodeHandle& getNodeHandle()
  {
    return nh_;
  }

  /*! Set the ros node handle associated with this shared module.
    * @param nodeHandle  the ros nodehandle to be set
    */
  void setNodeHandle(const ros::NodeHandle& nodeHandle)
  {
    nh_ = nodeHandle;
  }

 private:
  // ros node handle
  ros::NodeHandle nh_;

};

using SharedModuleRosPtr = std::shared_ptr<SharedModuleRos>;


} /* namespace roco */
