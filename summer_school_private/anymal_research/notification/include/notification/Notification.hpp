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
* @file     Notification.hpp
* @author   Christian Gehring
* @date     May 21, 2015
*/

#pragma once


// ros
#include <ros/ros.h>

// notification_msgs
#include <notification_msgs/Notification.h>

// notification
#include "notification/Level.hpp"


namespace notification {


class Notification
{
public:
  //! ROS header.
  std_msgs::Header header_;

  //! Code.
  unsigned int code_;

  //! Name.
  std::string name_;

  //! Description.
  std::string description_;

  //! Output devices.
  std::vector<std::string> outputDevices_;

  //! Identifier.
  std::string id_;

protected:
  //! Severity level.
  Level level_;


public:
  /*!
   * Constructor.
   * @param header ROS header.
   * @param code code.
   * @param name name.
   * @param description description.
   * @param outputDevices output devices.
   */
  Notification(const std_msgs::Header& header,
               Level level,
               unsigned int code,
               const std::string& name,
               const std::string& description,
               const std::vector<std::string>& outputDevices,
               const std::string& id);

  /*!
   * Constructor from ROS message.
   * @param msg ROS notification message.
   */
  Notification(const notification_msgs::Notification& msg);

  /*!
   * Destructor.
   */
  virtual ~Notification();

  /*!
   * Convert a ROS msg to a notification.
   * @param msg ROS msg.
   */
  void fromRosMsg(const notification_msgs::Notification& msg);

  /*!
   * Convert the notification to a ROS msg.
   * @return ROS notification msg.
   */
  notification_msgs::Notification toRosMsg() const;

  /*!
   * Set the notification level.
   * @param level notification level.
   */
  void setLevel(Level level);

  /*!
   * Get the notification level.
   * @return notification level.
   */
  Level getLevel() const;

  /*!
   * Get the notification level as unsigned char.
   * @return notification level as unsigned char.
   */
  unsigned char getLevelAsUnsignedChar() const;

  /*!
   * Get the notification level as string.
   * @return notification level as string.
   */
  std::string getLevelAsString() const;
};


std::ostream& operator<<(std::ostream& stream, const Notification& notification);


} // notification
