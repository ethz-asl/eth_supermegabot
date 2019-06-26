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

#ifndef FILEHANDLER_HPP_
#define FILEHANDLER_HPP_

#include <fstream>
#include <iostream>
#include <string>

#include <boost/asio.hpp>
#include <boost/smart_ptr.hpp>

#include "communication_layers/ssh_connections.hpp"
#include "config/config.hpp"

namespace visensor {

class DSO_EXPORT FileTransfer
{
 public:
  typedef boost::shared_ptr<FileTransfer> Ptr;
  FileTransfer(SshConnection::Ptr connection);
  virtual ~FileTransfer();

  virtual void downloadFile(const std::string& local_path, const std::string& remote_path);
  virtual void uploadFile(const std::string& local_path, const std::string& remote_path,
                          bool remount = false);
  virtual void readRemoteFile(const std::string& remote_path, std::string* file_content);
  virtual void writeRemoteFile(const std::string& remote_path, const std::string& file_content,
                               bool remount = false);
  void deleteRemoteFile(const std::string& remote_file, bool remount = false);

 private:
  void setFileMountRW(bool RW);

  SshConnection::Ptr ssh_connection_;
};

}

#endif /* FILEHANDLER_HPP_ */
