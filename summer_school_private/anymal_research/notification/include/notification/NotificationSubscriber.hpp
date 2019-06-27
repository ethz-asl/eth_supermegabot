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
* @file     NotificationSubscriber.hpp
* @author   Markus Zahner
* @date     May 27, 2015
*/

#pragma once


// c++
#include <cstdint>
#include <string>

// ros
#include <ros/ros.h>

// boost
#include <boost/function.hpp>

// notification
#include "notification/Notification.hpp"


namespace notification {


class NotificationSubscriber
{
public:
  typedef std::list<Notification> Notifications;


protected:
  //! Name of the notification subscriber.
  std::string outputDevice_;

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Function which is called when a notification is received which matches the output device.
  boost::function<void(const Notification&)> matchingNotificationCallback_;

  //! ROS notification subscriber.
  ros::Subscriber subscriber_;

  //! Notification priority buffer and size.
  bool usePriorityBuffer_;
  Notifications priorityBuffer_;
  unsigned int priorityBufferSize_;

  //! Top priority autoremove durations and timer.
  std::map<Level, double> autoremoveTopPriorityDurations_;
  ros::Timer autoremoveTopPriorityTimer_;

  //! Function which is called when the top priority notification changes.
  boost::function<void(const Notification&)> newTopPriorityCallback_;


public:
  /*!
   * Constructor.
   * @param outputDevice name of this notification subscriber.
   * @param nodeHandle ROS node handle.
   * @param matchingNotificationCallback function which is called when a notification is received which matches the output device.
   * @param priorityBufferSize size of the priority buffer, if it is 0 no priority buffer will be used.
   * @param newTopPriorityCallback function which is called when the top priority notification changes.
   */
  NotificationSubscriber(const std::string& outputDevice,
                         ros::NodeHandle& nodeHandle,
                         boost::function<void(const Notification&)> matchingNotificationCallback = 0,
                         unsigned int priorityBufferSize = 0,
                         boost::function<void(const Notification&)> newTopPriorityCallback = 0);

  /*!
   * Destructor.
   */
  virtual ~NotificationSubscriber();

  /*!
   * Set the duration how long the notification of a certain level should remain at top priority of the priority buffer.
   * @param level notification level.
   * @param duration duration for the given level.
   */
  void setAutoremoveTopPriorityDuration(Level level, double duration);

  /*!
   * Insert a new notification in the priority buffer.
   * @param notification notification.
   */
  void insertNewNotification(const Notification& notification);

  /*!
   * Remove the top priority notification from the priority buffer.
   */
  void removeTopPriorityNotification();

  /*!
   * Get the top notifications of the priority buffer.
   * @param numberOfNotifications number of notifications.
   * @return top priority notifications.
   */
  Notifications getTopPriorityNotifications(long unsigned int numberOfNotifications) const;

  /*!
   * Print the priority buffer content.
   */
  void printPriorityBuffer() const;

  /*!
   * Shut the subscriber down.
   */
  void shutdown();


protected:
  /*!
   * Internal notification callback, which checks if this subscriber was adressed and calls the custom callback.
   * @param msg notification message.
   */
  void notificationCallback(const notification_msgs::NotificationConstPtr& msg);

  /*!
   * Callback function for the autoremove top priority timer.
   */
  void autoremoveTopPriorityTimerCallback(const ros::TimerEvent&);

  /*!
   * Set up timer and trigger callback function for a new top priority notification.
   */
  void processNewTopPriorityNotification();
};


} // notification
