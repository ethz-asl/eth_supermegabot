/**
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, Christian Gehring
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
* @file     Notification.cpp
* @author   Christian Gehring
* @date     May 21, 2015
*/

// notification
#include "notification/Notification.hpp"


namespace notification {


Notification::Notification(const std_msgs::Header& header,
                           Level level,
                           unsigned int code,
                           const std::string& name,
                           const std::string& description,
                           const std::vector<std::string>& outputDevices,
                           const std::string& id)
: header_(header),
  code_(code),
  name_(name),
  description_(description),
  outputDevices_(outputDevices),
  id_(id)
{
  setLevel(level);
}


Notification::Notification(const notification_msgs::Notification& msg) {
  fromRosMsg(msg);
}


Notification::~Notification() {

}


void Notification::fromRosMsg(const notification_msgs::Notification& msg) {
  header_ = msg.header;
  setLevel(static_cast<Level>(msg.level));
  code_ = msg.code;
  name_ = msg.name;
  description_ = msg.description;
  outputDevices_ = msg.output_devices;
  id_ =  msg.id;
}


notification_msgs::Notification Notification::toRosMsg() const {
  notification_msgs::Notification notificationMsg;
  notificationMsg.header = header_;
  notificationMsg.level = getLevelAsUnsignedChar();
  notificationMsg.code = code_;
  notificationMsg.name = name_;
  notificationMsg.description = description_;
  notificationMsg.output_devices = outputDevices_;
  notificationMsg.id = id_;
  return notificationMsg;
}


void Notification::setLevel(Level level) {
  level_ = std::min(level, static_cast<Level>(static_cast<unsigned char>(Level::LEVEL_MAX) - 1));
}


Level Notification::getLevel() const {
  return level_;
}


unsigned char Notification::getLevelAsUnsignedChar() const {
  return static_cast<unsigned char>(level_);
}


std::string Notification::getLevelAsString() const {
  std::string levelAsString;
  switch (level_) {
    case Level::LEVEL_DEBUG:
      levelAsString = "DEBUG";
      break;
    case Level::LEVEL_INFO:
      levelAsString = "INFO";
      break;
    case Level::LEVEL_WARN:
      levelAsString = "WARN";
      break;
    case Level::LEVEL_ERROR:
      levelAsString = "ERROR";
      break;
    case Level::LEVEL_FATAL:
      levelAsString = "FATAL";
      break;
    default:
      levelAsString = "INVALID";
      break;
  }
  return levelAsString;
}


std::ostream& operator<<(std::ostream& stream, const Notification& notification) {
  stream << notification.getLevelAsString() << ": " << notification.name_ << " (" << notification.description_ << ")";
  return stream;
}


} // notification
