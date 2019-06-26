/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
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

#ifndef SSHCONNECTION_HPP_
#define SSHCONNECTION_HPP_

#include <string>

#include <boost/smart_ptr.hpp>
#include <libssh2.h>

#include "visensor/visensor.hpp"

namespace visensor {
class DSO_EXPORT SshConnection
{
 public:
  typedef boost::shared_ptr<SshConnection> Ptr;
  SshConnection(const unsigned int port = 22);

  virtual ~SshConnection();

  /* protocol-lvl functionality */
  bool runCommand(const std::string& command, std::string* output, int& exitcode) const;
  bool sendFile(const std::string& srcpath, const std::string& scppath) const;
  bool getFile(const std::string& scppath, const std::string& dstpath) const;
  bool loadFile(const std::string& scp_path, std::string* file_content) const;
  bool writeToFile(const std::string& scp_path, const std::string& file_content) const;

  /* ssh session stuff mgmt */
  bool sshConnect(const std::string& hostname, const std::string& username,
                  const std::string& password);
  void sshDisconnect();
  bool isOpen() const;

 private:
  LIBSSH2_SESSION* session_ssh_;   //ssh session
  unsigned int port_;
  int socket_;                     //network socket
};

}
#endif /* SSHCONNECTION_HPP_ */

