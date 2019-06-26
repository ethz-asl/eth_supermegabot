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

#include <fstream>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "networking/file_transfer.hpp"
#include "visensor/visensor_exceptions.hpp"

namespace visensor {

FileTransfer::FileTransfer(SshConnection::Ptr connection)
    : ssh_connection_(connection)
{
}

FileTransfer::~FileTransfer()
{
}

void FileTransfer::downloadFile(const std::string& local_path, const std::string& remote_path)
{
  // receive file and write to path
  if (!ssh_connection_->getFile(remote_path, local_path))
    throw visensor::exceptions::ConnectionException(
        "Could not downloading file from the sensor " + remote_path + " to " + local_path);
}

void FileTransfer::deleteRemoteFile(const std::string& remote_file, bool remount)
{
  std::string output;
  int exitcode = 127;

  // remount the sensor if needed
  setFileMountRW(remount);

  //delete existing file
  ssh_connection_->runCommand("ls " + remote_file, &output, exitcode);
  if (exitcode == 0) {
    ssh_connection_->runCommand("rm " + remote_file, &output, exitcode);
  }

  setFileMountRW(false);
}
void FileTransfer::uploadFile(const std::string& local_path, const std::string& remote_file,
                              bool remount)
{
  // get only the path without filename to create the folder if needed
  std::string remote_path = remote_file.substr(0, remote_file.find_last_of("\\/"));
  std::string output;
  int exitcode = 127;

  std::ifstream local_file;
  local_file.open(local_path.c_str());
  std::string local_file_content;
  local_file >> local_file_content;

  // remount the sensor if needed
  setFileMountRW(remount);

  ssh_connection_->runCommand("ls " + remote_path, &output, exitcode);
  if (exitcode != 0)
    ssh_connection_->runCommand("mkdir " + remote_path, &output, exitcode);
  if (exitcode)
    throw visensor::exceptions::ConnectionException(
        "Could not create directory " + remote_path + " on the sensor.");

  //delete existing file
  ssh_connection_->runCommand("ls " + remote_file, &output, exitcode);
  if (exitcode == 0) {
    ssh_connection_->runCommand("rm " + remote_file, &output, exitcode);
  }

  // read file from path and send it to the sensor
  if (!ssh_connection_->sendFile(local_path, remote_file))
    throw visensor::exceptions::ConnectionException(
        "Could not upload file " + local_path + " to the sensor " + remote_file);

  setFileMountRW(false);
}

void FileTransfer::readRemoteFile(const std::string &remote_path, std::string* file_content)
{
  if (!ssh_connection_->loadFile(remote_path, file_content))
    throw visensor::exceptions::ConnectionException(
        "Could not load file content from sensor file: " + remote_path);

}

void FileTransfer::writeRemoteFile(const std::string &remote_file, const std::string &file_content,
                                   bool remount)
{
  // get only the path without filename to create the folder if needed
  std::string remote_path = remote_file.substr(0, remote_file.find_last_of("\\/"));
  std::string output;
  int exitcode = 127;

  // remount the sensor if needed
  setFileMountRW(remount);

  // create folder if there is no
  ssh_connection_->runCommand("ls " + remote_path, &output, exitcode);
  if (exitcode != 0)
    ssh_connection_->runCommand("mkdir " + remote_path, &output, exitcode);
  if (exitcode)
    throw visensor::exceptions::ConnectionException(
        "Could not create directory " + remote_path + " on the sensor.");

  //delete existing file
  ssh_connection_->runCommand("ls " + remote_file, &output, exitcode);
  if (exitcode == 0) {
    ssh_connection_->runCommand("rm " + remote_file, &output, exitcode);
  }

  if (!ssh_connection_->writeToFile(remote_file, file_content))
    throw visensor::exceptions::ConnectionException(
        "Could not write file content to sensor file: " + remote_file);
  setFileMountRW(false);

}

//RW: true ==> read-write, RW: false ==> read-only
void FileTransfer::setFileMountRW(bool RW)
{

  std::string output;
  int exitcode = 127;

  /*command */
  std::string cmd;
  if (RW)
    cmd = "mount -o remount,rw /";
  else
    cmd = "mount -o remount,ro /";

  /* run command */
  ssh_connection_->runCommand(cmd, &output, exitcode);

  //0 and 255 are success exit codes
  if (exitcode != 0 && exitcode != 255)
    throw visensor::exceptions::ConnectionException("Could not change RW mode on sensor!\n");
}

}
