#pragma once
// qt
#include <QThread>
// ros
#include <ros/ros.h>
// msgs
#include <rocoma_msgs/SwitchController.h>

class WorkerThreadSetMode : public QThread {
  Q_OBJECT

  void run();

 public:
  void setClient(ros::ServiceClient& client);
  void setRequest(rocoma_msgs::SwitchControllerRequest request);

 private:
  rocoma_msgs::SwitchControllerRequest request_;
  rocoma_msgs::SwitchControllerResponse response_;
  ros::ServiceClient switchControllerModeClient_;

 signals:
  void resultReady(bool isOk, rocoma_msgs::SwitchControllerResponse response);
};
