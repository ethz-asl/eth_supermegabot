/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef VISENSOR_EXCEPTIONS_HPP_
#define VISENSOR_EXCEPTIONS_HPP_

#include <stdexcept>

#include "visensor/visensor_config.hpp"

namespace visensor {

class DSO_EXPORT exceptions : public std::runtime_error
{
  public:
  exceptions(std::string msg): std::runtime_error(msg) {}

  class ConnectionException;
  class SensorException;
  class FirmwareException;
  class ConfigException;
};

class DSO_EXPORT exceptions::ConnectionException : public exceptions
{
  public:
  ConnectionException(std::string msg): exceptions(msg) {}
};

class DSO_EXPORT exceptions::SensorException : public exceptions
{
  public:
  SensorException(std::string msg): exceptions(msg) {}
};

class DSO_EXPORT exceptions::FirmwareException : public exceptions
{
  public:
  FirmwareException(std::string msg): exceptions(msg) {}
};

class DSO_EXPORT exceptions::ConfigException : public exceptions
{
  public:
  ConfigException(std::string msg): exceptions(msg) {}
};


} //namespace visensor

#endif /* VISENSOR_EXCEPTIONS_HPP_ */
