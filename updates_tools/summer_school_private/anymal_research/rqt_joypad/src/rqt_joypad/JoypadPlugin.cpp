/*
 * LocoJoypadPlugin.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: C. Dario Bellicoso
 */

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QGridLayout>
#include <rqt_joypad/JoypadPlugin.hpp>

namespace rqt_joypad {

JoypadPlugin::JoypadPlugin() :
    rqt_gui_cpp::Plugin(),
    widget_(0) {
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("LocoVisualizerPlugin");

  int numButtons = 15;
  int numAxes = 7;
  for (int k = 0; k < numButtons; k++) {
    int value = 0;
    anyJoyMsg_.joy.buttons.push_back(value);
  }

  for (int k = 0; k < numAxes; k++) {
    int value = 0;
    anyJoyMsg_.joy.axes.push_back(value);
  }
  joyMsgRepeatTimer_ = new QTimer(widget_);
  joyMsgRepeatTimer_->setInterval(1000);

}


void JoypadPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
  // access standalone command line arguments
  QStringList argv = context.argv();

  // create the main widget, set it up and add it to the user interface
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

//    widget_->setStyleSheet(QString("background-image: url(/home/dario/work/catkin_ws/src/starleth_user_interface/rqt_joypad/resource/joypad.jpg)"));

  connect(ui_.pushButtonA, SIGNAL(pressed()), this, SLOT(buttonAPressedFcn()));
  connect(ui_.pushButtonB, SIGNAL(pressed()), this, SLOT(buttonBPressedFcn()));
  connect(ui_.pushButtonX, SIGNAL(pressed()), this, SLOT(buttonXPressedFcn()));
  connect(ui_.pushButtonY, SIGNAL(pressed()), this, SLOT(buttonYPressedFcn()));

  connect(ui_.pushButtonUp, SIGNAL(pressed()), this,
          SLOT(buttonUpPressedFcn()));
  connect(ui_.pushButtonDown, SIGNAL(pressed()), this,
          SLOT(buttonDownPressedFcn()));
  connect(ui_.pushButtonLeft, SIGNAL(pressed()), this,
          SLOT(buttonLeftPressedFcn()));
  connect(ui_.pushButtonRight, SIGNAL(pressed()), this,
          SLOT(buttonRightPressedFcn()));

  connect(ui_.pushButtonA, SIGNAL(released()), this,
          SLOT(buttonAReleasedFcn()));
  connect(ui_.pushButtonB, SIGNAL(released()), this,
          SLOT(buttonBReleasedFcn()));
  connect(ui_.pushButtonX, SIGNAL(released()), this,
          SLOT(buttonXReleasedFcn()));
  connect(ui_.pushButtonY, SIGNAL(released()), this,
          SLOT(buttonYReleasedFcn()));

  connect(ui_.pushButtonUp, SIGNAL(released()), this,
          SLOT(buttonUpReleasedFcn()));
  connect(ui_.pushButtonDown, SIGNAL(released()), this,
          SLOT(buttonDownReleasedFcn()));
  connect(ui_.pushButtonLeft, SIGNAL(released()), this,
          SLOT(buttonLeftReleasedFcn()));
  connect(ui_.pushButtonRight, SIGNAL(released()), this,
          SLOT(buttonRightReleasedFcn()));

  connect(ui_.pushButtonResetAxes, SIGNAL(pressed()), this,
          SLOT(buttonResetAxesPressed()));

  connect(ui_.axisXLeft, SIGNAL(valueChanged(int)), this,
          SLOT(axisXLeftMoved()));
  connect(ui_.axisYLeft, SIGNAL(valueChanged(int)), this,
          SLOT(axisYLeftMoved()));
  connect(ui_.axisXRight, SIGNAL(valueChanged(int)), this,
          SLOT(axisXRightMoved()));
  connect(ui_.axisYRight, SIGNAL(valueChanged(int)), this,
          SLOT(axisYRightMoved()));

  connect(ui_.pushButtonStickReset, SIGNAL(toggled(bool)), this,
          SLOT(buttonStickResetFcn(bool)));

  connect(this, SIGNAL(stateChanged()), this, SLOT(publishAnyJoyMessage()));
  connect(this, SIGNAL(stateChanged()), joyMsgRepeatTimer_, SLOT(start()));
  connect(joyMsgRepeatTimer_, SIGNAL(timeout()), this, SLOT(publishAnyJoyMessage()));
  std::string topic("");
  getNodeHandle().param<std::string>("publishers/joyPub/topic", topic, "/anyjoy/rqt");
  joyPub_ = getNodeHandle().advertise<joy_manager_msgs::AnyJoy>(topic, 0);

  // reset joystick axes
  buttonResetAxesPressed();

  // hide slider "joystick"
  ui_.frame_joystick_slider->hide();

  // add joystick left
  joystick_left_ = new Joystick();
  ui_.joystick_left->addWidget(joystick_left_);
  connect(joystick_left_, SIGNAL(joystickMoved(double, double)),
          this, SLOT(joystickLeftMoved(double, double)));
  // add joystick right
  joystick_right_ = new Joystick();
  ui_.joystick_right->addWidget(joystick_right_);
  connect(joystick_right_, SIGNAL(joystickMoved(double, double)),
          this, SLOT(joystickRightMoved(double, double)));
}

void JoypadPlugin::joystickLeftMoved(double x, double y) {

  anyJoyMsg_.joy.axes.at(JoyAxes::Lateral) = x;
  anyJoyMsg_.joy.axes.at(JoyAxes::Heading) = y;
  emit stateChanged();
}

void JoypadPlugin::joystickRightMoved(double x, double y) {

  anyJoyMsg_.joy.axes.at(JoyAxes::Yaw) = x;
  anyJoyMsg_.joy.axes.at(JoyAxes::Pitch) = y;
  anyJoyMsg_.joy.axes.at(JoyAxes::Vertical) = y;
  emit stateChanged();
}

void JoypadPlugin::publishAnyJoyMessage() {
  anyJoyMsg_.header.stamp = ros::Time::now();
  joyPub_.publish(anyJoyMsg_);
  anyJoyMsg_.commands = {};
  anyJoyMsg_.modules = {};
  //joyLocoPub_.publish(joyMsg_);
}


void JoypadPlugin::buttonEmergencyStopPressed() {
  buttonResetAxesPressed();
  anyJoyMsg_.joy.buttons.at(JoyButtons::EmergencyStop) = 1;
  anyJoyMsg_.commands = {"soft_emcy_stop"};
  emit stateChanged();
}


void JoypadPlugin::buttonEmergencyStopReleased() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::EmergencyStop) = 0;
  emit stateChanged();
}


void JoypadPlugin::buttonResetAxesPressed() {
  ui_.axisXLeft->setValue(0);
  ui_.axisYLeft->setValue(0);
  ui_.axisXRight->setValue(0);
  ui_.axisYRight->setValue(0);
  emit stateChanged();
}

/***************************
 * X,Y axes: value changed *
 ***************************/
void JoypadPlugin::axisXLeftMoved() {
  anyJoyMsg_.joy.axes.at(JoyAxes::Lateral) =
      -(double)ui_.axisXLeft->value() / (double)ui_.axisXLeft->maximum();
  emit stateChanged();
}

void JoypadPlugin::axisYLeftMoved() {
  anyJoyMsg_.joy.axes.at(JoyAxes::Heading) =
      (double)ui_.axisYLeft->value() / (double)ui_.axisYLeft->maximum();
  emit stateChanged();
}

void JoypadPlugin::axisXRightMoved() {
  anyJoyMsg_.joy.axes.at(JoyAxes::Yaw) =
      -(double)ui_.axisXRight->value() / (double)ui_.axisXRight->maximum();
  emit stateChanged();
}

void JoypadPlugin::axisYRightMoved() {
  anyJoyMsg_.joy.axes.at(JoyAxes::Pitch) =
      (double)ui_.axisYRight->value() / (double)ui_.axisYRight->maximum();
  anyJoyMsg_.joy.axes.at(JoyAxes::Vertical) =
      (double)ui_.axisYRight->value() / (double)ui_.axisYRight->maximum();
  emit stateChanged();
}
/***************************/


/*********************
 * A,B,X,Y: released *
 *********************/
void JoypadPlugin::buttonAReleasedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonA) = 0;
  emit stateChanged();
}

void JoypadPlugin::buttonBReleasedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonB) = 0;
  emit stateChanged();
}

void JoypadPlugin::buttonXReleasedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonX) = 0;
  emit stateChanged();
}

void JoypadPlugin::buttonYReleasedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonY) = 0;
  emit stateChanged();
}
/*********************/


/********************
 * A,B,X,Y: pressed *
 ********************/
void JoypadPlugin::buttonAPressedFcn() {
  buttonResetAxesPressed();
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonA) = 1;
  emit stateChanged();
}

void JoypadPlugin::buttonBPressedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonB) = 1;
  emit stateChanged();
}

void JoypadPlugin::buttonXPressedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonX) = 1;
  emit stateChanged();
}

void JoypadPlugin::buttonYPressedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonY) = 1;
  emit stateChanged();
}

/********************/








void JoypadPlugin::buttonUpPressedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonUp) = 1;
  emit stateChanged();
}

void JoypadPlugin::buttonDownPressedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonDown) = 1;
  emit stateChanged();
}

void JoypadPlugin::buttonLeftPressedFcn() {

}

void JoypadPlugin::buttonRightPressedFcn() {

}

void JoypadPlugin::buttonUpReleasedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonUp) = 0;
  emit stateChanged();
}

void JoypadPlugin::buttonDownReleasedFcn() {
  anyJoyMsg_.joy.buttons.at(JoyButtons::ButtonDown) = 0;
  emit stateChanged();
}

void JoypadPlugin::buttonLeftReleasedFcn() {

}

void JoypadPlugin::buttonRightReleasedFcn() {

}


void JoypadPlugin::buttonStickResetFcn(bool checked) {
  joystick_left_->setAutoReset(checked);
  joystick_right_->setAutoReset(checked);
}


void JoypadPlugin::shutdownPlugin() {
  // TODO unregister all publishers here
  buttonResetAxesPressed();
  joyPub_.shutdown();
  //joyLocoPub_.shutdown();
}


void JoypadPlugin::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                qt_gui_cpp::Settings &instance_settings) const {
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}


void JoypadPlugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                   const qt_gui_cpp::Settings &instance_settings) {
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}


/*bool hasConfiguration() const
{
  return true;
}


void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_joypad::JoypadPlugin, rqt_gui_cpp::Plugin)
