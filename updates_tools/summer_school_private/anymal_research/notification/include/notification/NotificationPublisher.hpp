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
* @file     NotificationPublisher.hpp
* @author   Christian Gehring
* @date     May 21, 2015
*/

#pragma once


// c++
#include <list>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>

// notification
#include "notification/Notification.hpp"


namespace notification {


class NotificationPublisher
{
public:
  typedef std::list<Notification> Notifications;

protected:
  //! Name of the notification publisher.
  std::string name_;

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! If publish immediately is enabled, notifications will be published immediately with notify(), otherwise publish() has to be called.
  bool publishImmediately_;

  //! ROS notification publisher.
  ros::Publisher publisher_;

  //! Vector storing the names of all output devices.
  std::vector<std::string> outputDevices_;

  //! Notification buffer, storing unpulished notifications.
  Notifications notificationBuffer_;

public:
  /*!
   * Constructor.
   * @param name name of the notification publisher.
   * @param nodeHandle ROS node handle.
   * @param publishImmediately
   */
  NotificationPublisher(const std::string& name, ros::NodeHandle& nodeHandle, bool publishImmediately = true);

  /*!
   * Destructor.
   */
  virtual ~NotificationPublisher();

  /*!
   * Send a notification. If publish immediately is enabled, the notification is sent immediately.
   * @param level level of severity.
   * @param name name of the notification.
   * @param description description of the notification.
   * @param code code of the notification.
   * @param id    identifier
   */
  void notify(Level level,
              const std::string& name,
              const std::string& description = std::string(),
              const std::string& id = std::string(),
              unsigned int code = 0);

  /*!
   * Send a notification. If publish immediately is enabled, the notification is sent immediately.
   * @param level level of severity.
   * @param name name of the notification.
   * @param description description of the notification.
   * @param code code of the notification.
   * @param id    identifier
   * @param outputDevices   non-standard output devices
   */
  void notify(Level level,
               const std::string& name,
               const std::string& description,
               const std::string& id,
               unsigned int code,
               const std::vector<std::string>& outputDevices);

  /*!
   * Publish the notifications in the buffer manually and clear it afterwards. If publish immediately is disabled, this method needs to be invoked to actually send the notifications.
   */
  void publish();

  /*!
   * Print the notification buffer content.
   */
  void printNotificationBuffer() const;
};


} // notification
