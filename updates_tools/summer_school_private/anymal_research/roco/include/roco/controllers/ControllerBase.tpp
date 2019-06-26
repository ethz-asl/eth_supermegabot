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

namespace roco {

template<typename State_, typename Command_, class ...Interfaces_>
ControllerBase<State_, Command_, Interfaces_...>::ControllerBase():
  name_(""),
  parameterPath_(""),
  isCreated_(false),
  isInitialized_(false),
  isRunning_(false)
{

}

template<typename State_, typename Command_, class... Interfaces_>
ControllerBase<State_, Command_, Interfaces_...>::~ControllerBase()
{

}

template<typename State_, typename Command_, class... Interfaces_>
const std::string& ControllerBase<State_, Command_, Interfaces_...>::getName() const {
  return name_;
}

template<typename State_, typename Command_, class... Interfaces_>
void ControllerBase<State_, Command_, Interfaces_...>::setName(const std::string& name) {
  name_ = name;
}

template<typename State_, typename Command_, class... Interfaces_>
const std::string& ControllerBase<State_, Command_, Interfaces_...>::getParameterPath() const {
  return parameterPath_;
}

template<typename State_, typename Command_, class... Interfaces_>
void ControllerBase<State_, Command_, Interfaces_...>::setParameterPath(const std::string& parameterPath) {
  parameterPath_ = parameterPath;
}

template<typename State_, typename Command_, class... Interfaces_>
bool ControllerBase<State_, Command_, Interfaces_...>::isInitialized() const {
  return isInitialized_;
}

template<typename State_, typename Command_, class... Interfaces_>
bool ControllerBase<State_, Command_, Interfaces_...>::isCreated() const {
  return isCreated_;
}

template<typename State_, typename Command_, class... Interfaces_>
bool ControllerBase<State_, Command_, Interfaces_...>::isRunning() const {
  return isRunning_;
}

} /* namespace roco */
