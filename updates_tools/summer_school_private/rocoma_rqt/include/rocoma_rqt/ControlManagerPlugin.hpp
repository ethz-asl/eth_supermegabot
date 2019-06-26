/*
 * ControlManagerPlugin.hpp
 *
 *  Created on: May 2019
 *      Author: Johannes Pankert
 */

#pragma once

#include <rocoma_rqt/WorkerThreadSetMode.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_rocoma_rqt_plugin.h>
#include <QWidget>

#include <rocoma_msgs/GetActiveController.h>
#include <rocoma_msgs/GetAvailableControllers.h>
#include <rocoma_msgs/SwitchController.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <message_logger/message_logger.hpp>

class ControlManagerPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT

 public:
  ControlManagerPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
 private:
  Ui::ControlManager ui_;
  QWidget* widget_;
  ros::ServiceClient controllerClient_;

  //  ros::ServiceClient switchLowLevelControllerClient_;
  ros::ServiceClient getAvailableControllersClient_;
  ros::ServiceClient getActiveControllerClient_;
  ros::ServiceClient emergencyStopClient_;

  ros::Subscriber activeControllerSubscriber_;

  std::string highlevelControllerNamespace_;

  virtual std::string getStatusName(rocoma_msgs::SwitchController::Response& res);

 protected slots:

  virtual void buttonEmergencyStopPressed();

  virtual void buttonRefreshPressed();

  virtual void buttonSetPressed();

  virtual void comboBoxActiveControllersActivated(QString text);

  virtual void connectServiceClients();

  void receiveActiveControllerChanged(QString activeControllerName);

 signals:

  void activeControllerChanged(QString activeControllerName);

  void stateChanged();

 protected:
  void activeControllerCallback(const std_msgs::StringConstPtr& msg);
};
