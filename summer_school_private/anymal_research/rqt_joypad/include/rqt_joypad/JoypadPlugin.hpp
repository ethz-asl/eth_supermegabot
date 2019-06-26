/*
 * LocoJoypadPlugin.hpp
 *
 *  Created on: Jan 19, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

#include "joystick_label/Joystick.h"

#include <rqt_gui_cpp/plugin.h>
#include <ui_joypad_plugin.h>
#include <QWidget>
#include <QLabel>
#include <QTimer>

#include <ros/ros.h>
#include <ros/package.h>
#include <joy_manager_msgs/AnyJoy.h>

using namespace joystick_label;

namespace rqt_joypad {

class JoypadPlugin : public rqt_gui_cpp::Plugin {
Q_OBJECT

public:

  enum JoyAxes {
    Heading = 1,
    Lateral = 0,
    Yaw = 3,
    Roll = 3,
    Pitch = 2,
    Vertical = 4
  };

  enum JoyButtons {
    EmergencyStop = 5,
    ButtonA = 0,
    ButtonB = 1,
    ButtonX = 2,
    ButtonY = 3,
    ButtonUp = 13,
    ButtonDown = 14
  };

  JoypadPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext &context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                            qt_gui_cpp::Settings &instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                               const qt_gui_cpp::Settings &instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private:
  Ui::LocoJoypad ui_;
  QWidget *widget_;

  ros::Publisher joyPub_;
  //ros::Publisher joyLocoPub_;
  joy_manager_msgs::AnyJoy anyJoyMsg_;
  QTimer *joyMsgRepeatTimer_;

  ros::ServiceClient controllerSrv_;

  Joystick *joystick_left_;
  Joystick *joystick_right_;

protected slots:

  virtual void buttonAReleasedFcn();

  virtual void buttonBReleasedFcn();

  virtual void buttonXReleasedFcn();

  virtual void buttonYReleasedFcn();

  virtual void buttonAPressedFcn();

  virtual void buttonBPressedFcn();

  virtual void buttonXPressedFcn();

  virtual void buttonYPressedFcn();

  virtual void buttonUpPressedFcn();

  virtual void buttonDownPressedFcn();

  virtual void buttonLeftPressedFcn();

  virtual void buttonRightPressedFcn();

  virtual void buttonUpReleasedFcn();

  virtual void buttonDownReleasedFcn();

  virtual void buttonLeftReleasedFcn();

  virtual void buttonRightReleasedFcn();

  virtual void buttonStickResetFcn(bool checked);

  virtual void axisXLeftMoved();

  virtual void axisYLeftMoved();

  virtual void axisXRightMoved();

  virtual void axisYRightMoved();

  virtual void buttonResetAxesPressed();

  virtual void buttonEmergencyStopPressed();

  virtual void buttonEmergencyStopReleased();

  virtual void joystickLeftMoved(double x, double y);

  virtual void joystickRightMoved(double x, double y);

  virtual void publishAnyJoyMessage();

signals:

  void stateChanged();
};

} // namespace
