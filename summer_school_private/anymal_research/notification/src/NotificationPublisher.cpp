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
* @file     NotificationPublisher.cpp
* @author   Christian Gehring
* @date     May 21, 2015
*/

// notification
#include "notification/NotificationPublisher.hpp"


namespace notification {


NotificationPublisher::NotificationPublisher(const std::string& name, ros::NodeHandle& nodeHandle, bool publishImmediately)
: name_(name),
  nodeHandle_(nodeHandle),
  publishImmediately_(publishImmediately)
{
  // Define the parameter namespace.
  const std::string parameterNamespace = "notification/notifiers/" + name_ + "/";

  // Create publisher.
  std::string topic = "notification";
  if (!nodeHandle_.getParam(parameterNamespace + "topic", topic)) {
    ROS_WARN_STREAM("Could not get topic from parameter server for notifier " << name << ".");
  }
  bool latch = false;
  if (!nodeHandle_.getParam(parameterNamespace + "latch", latch)) {
    ROS_WARN_STREAM("Could not get latch from parameter server for notifier " << name << ".");
  }
  int queueSize = 100;
  if (!nodeHandle_.getParam(parameterNamespace + "queue_size", queueSize)) {
    ROS_WARN_STREAM("Could not get queue size from parameter server for notifier " << name << ".");
  }
  publisher_ = nodeHandle_.advertise<notification_msgs::Notification>(topic, queueSize, latch);

  // Read list of output devices.
  if (!nodeHandle_.getParam(parameterNamespace + "output_devices", outputDevices_)) {
    ROS_INFO_STREAM("Could not get output devices from parameter server for notifier " << name << ", will broadcast.");
  }
}


NotificationPublisher::~NotificationPublisher() {
  publisher_.shutdown();
}


void NotificationPublisher::notify(Level level,
                                   const std::string& name,
                                   const std::string& description,
                                   const std::string& id,
                                   unsigned int code) {
  // Create a notification.
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  Notification notification(header, level, code, name, description, outputDevices_, id);

  // Add the notification to the buffer.
  notificationBuffer_.push_back(notification);

  // Publish if publish immediately is enabled.
  if (publishImmediately_) {
    publish();
  }
}

void NotificationPublisher::notify(Level level,
                                   const std::string& name,
                                   const std::string& description,
                                   const std::string& id,
                                   unsigned int code,
                                   const std::vector<std::string>& outputDevices) {
  // Create a notification.
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  Notification notification(header, level, code, name, description, outputDevices, id);

  // Add the notification to the buffer.
  notificationBuffer_.push_back(notification);

  // Publish if publish immediately is enabled.
  if (publishImmediately_) {
    publish();
  }
}


void NotificationPublisher::publish() {
  // Publish if subscribers are around.
  if (publisher_.getNumSubscribers() > 0u) {
    for (const Notification& notfication : notificationBuffer_) {
      publisher_.publish(notfication.toRosMsg());
    }
  }

  // Clear the buffer whether subscribers were around or not.
  notificationBuffer_.clear();
}


void NotificationPublisher::printNotificationBuffer() const {
  // Print the notifications in the buffer.
  std::cout << "Current notification buffer:" << std::endl;
  for (const Notification& notification : notificationBuffer_) {
    std::cout << notification << std::endl;
  }
}


} // notification
