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
* @file     NotificationSubscriber.cpp
* @author   Markus Zahner
* @date     May 21, 2015
*/

// notification
#include "notification/NotificationSubscriber.hpp"


namespace notification {


NotificationSubscriber::NotificationSubscriber(const std::string& outputDevice,
                                               ros::NodeHandle& nodeHandle,
                                               boost::function<void(const Notification&)> matchingNotificationCallback,
                                               unsigned int priorityBufferSize,
                                               boost::function<void(const Notification&)> newTopPriorityCallback)
: outputDevice_(outputDevice),
  nodeHandle_(nodeHandle),
  matchingNotificationCallback_(matchingNotificationCallback),
  priorityBufferSize_(priorityBufferSize),
  newTopPriorityCallback_(newTopPriorityCallback)
{
  // Define the parameter namespace.
  const std::string parameterNamespace = "notification/output_devices/" + outputDevice_ + "/";

  // Create subscriber.
  std::string topic = "/notification";
  if (!nodeHandle_.getParam(parameterNamespace + "topic", topic)) {
    ROS_WARN_STREAM("Could not get topic from parameter server for output device " << outputDevice << ".");
  }
  int queueSize = 100;
  if (!nodeHandle_.getParam(parameterNamespace + "queue_size", queueSize)) {
    ROS_WARN_STREAM("Could not get queue size from parameter server for output device " << outputDevice << ".");
  }
  subscriber_ = nodeHandle_.subscribe(topic, queueSize, &NotificationSubscriber::notificationCallback, this);


  // Decide whether to use a priority buffer.
  usePriorityBuffer_ = (priorityBufferSize > 0);

  if (usePriorityBuffer_) {
    // Initialize the durations to remove the top priority notification of a certain level.
    for (unsigned int level = 0; level < static_cast<unsigned int>(Level::LEVEL_MAX); level++) {
      autoremoveTopPriorityDurations_.insert(std::make_pair(static_cast<Level>(level), level + 1));
    }

    // Initialize a timer, but do not start it.
    autoremoveTopPriorityTimer_ = nodeHandle_.createTimer(ros::Duration(1.0), &NotificationSubscriber::autoremoveTopPriorityTimerCallback, this, true, false);
  }
}


NotificationSubscriber::~NotificationSubscriber() {

}


void NotificationSubscriber::setAutoremoveTopPriorityDuration(Level level, double duration) {
  // Check if priority buffer is enabled.
  if (!usePriorityBuffer_) {
    ROS_ERROR_STREAM("Priority buffer is disabled, cannot set autoremove top priority duration.");
    return;
  }

  autoremoveTopPriorityDurations_.at(level) = duration;
}


void NotificationSubscriber::insertNewNotification(const Notification& notification) {
  // Check if priority buffer is enabled.
  if (!usePriorityBuffer_) {
    ROS_ERROR_STREAM("Priority buffer is disabled, cannot insert new notification.");
    return;
  }

  // Find the place to insert the notification.
  Notifications::iterator it = priorityBuffer_.begin();
  bool newTopPriorityNotification = false;
  while (it != priorityBuffer_.end()) {
    if (it->getLevel() < notification.getLevel()) {
      // We have to insert here.
      break;
    }
    ++it;
  }
  if (it == priorityBuffer_.begin()) {
    // The buffer has a new top priority notification.
    newTopPriorityNotification = true;
  }
  priorityBuffer_.insert(it, notification);

  // If we have too many objects, remove the oldest of lowest priority.
  if (priorityBuffer_.size()>priorityBufferSize_) {
    Level lowestLevel = priorityBuffer_.back().getLevel();
    Notifications::iterator oldestOfLowestLevel = priorityBuffer_.end();
    while(oldestOfLowestLevel != priorityBuffer_.begin()) {
      --oldestOfLowestLevel;
      if (oldestOfLowestLevel->getLevel() > lowestLevel) {
        ++oldestOfLowestLevel;
        break;
      }
    }
    priorityBuffer_.erase(oldestOfLowestLevel);
  }

  if (newTopPriorityNotification) {
    // The buffer has a new top priority notification.
    autoremoveTopPriorityTimer_.stop();
    processNewTopPriorityNotification();
  }

  // Print the updated priority buffer.
//  printPriorityBuffer();
}


void NotificationSubscriber::removeTopPriorityNotification() {
  // Check if priority buffer is enabled.
  if (!usePriorityBuffer_) {
    ROS_ERROR_STREAM("Priority buffer is disabled, cannot remove top priority notification.");
    return;
  }

  // Stop the timer and remove the first element in the buffer.
  autoremoveTopPriorityTimer_.stop();
  if (priorityBuffer_.size() > 0) {
    priorityBuffer_.pop_front();
  }

  // Process the new top priority notification.
  processNewTopPriorityNotification();
}


NotificationSubscriber::Notifications NotificationSubscriber::getTopPriorityNotifications(long unsigned int numberOfNotifications) const {
  // Reduce the number of notifications to the maximum priority buffer size if it is bigger.
  if (numberOfNotifications > priorityBufferSize_) {
    ROS_WARN_STREAM("The required number of notifications exceeds the priority buffer size, reducing it.");
    numberOfNotifications = priorityBufferSize_;
  }

  // Make sure the number of notifications is not bigger than the current priority buffer size.
  numberOfNotifications = std::min(numberOfNotifications, priorityBuffer_.size());

  // Copy the highest priority notifications.
  Notifications topPriorityNotifications;
  Notifications::const_iterator it = priorityBuffer_.begin();
  while (topPriorityNotifications.size() < numberOfNotifications) {
    topPriorityNotifications.push_back(*it);
    ++it;
  }

  return topPriorityNotifications;
}


void NotificationSubscriber::printPriorityBuffer() const {
  // Check if priority buffer is enabled.
  if (!usePriorityBuffer_) {
    ROS_ERROR_STREAM("Priority buffer is disabled, cannot print it.");
    return;
  }

  // Print the notifications in the priority buffer.
  std::cout << "Current priority buffer:" << std::endl;
  for (const Notification& notification : priorityBuffer_) {
    std::cout << notification << std::endl;
  }
}


void NotificationSubscriber::shutdown() {
  subscriber_.shutdown();
}


void NotificationSubscriber::notificationCallback(const notification_msgs::NotificationConstPtr& msg) {
  bool notificationMatches = false;

  // Broadcast if the list of output devices is empty.
  if (msg->output_devices.size() == 0) {
    notificationMatches = true;
  } else {
    // Otherwise check if this subscriber was adressed by name.
    for (const std::string& outputDevice : msg->output_devices) {
      if (outputDevice == outputDevice_) {
        notificationMatches = true;
      }
    }
  }

  // If the notification matches this output device, trigger the callback and insert it into the priority buffer.
  if (notificationMatches) {
    Notification notification(*msg);
    if (matchingNotificationCallback_) {
      matchingNotificationCallback_(notification);
    }
    if (usePriorityBuffer_) {
      insertNewNotification(notification);
    }
  }
}


void NotificationSubscriber::autoremoveTopPriorityTimerCallback(const ros::TimerEvent&) {
  removeTopPriorityNotification();
}


void NotificationSubscriber::processNewTopPriorityNotification() {
  // Make sure the priority buffer is not empty.
  if(priorityBuffer_.size() > 0) {
    // Start the autoremove timer.
    autoremoveTopPriorityTimer_.setPeriod(ros::Duration(autoremoveTopPriorityDurations_.at(priorityBuffer_.front().getLevel())));
    autoremoveTopPriorityTimer_.start();

    // Execute the callback function for the new top priority notification.
    if (newTopPriorityCallback_) {
      newTopPriorityCallback_(priorityBuffer_.front());
    }
  }
}


} // notification
