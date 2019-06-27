/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <QLabel>
#include <QEvent>
#include <QMouseEvent>
#include <QApplication>

// TODO missing features (resizable)

namespace joystick_label {

class Joystick : public QLabel {
Q_OBJECT
public:

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  /**
   * @brief Initialize joystick.
   * @param parent
   */
  Joystick(QWidget *parent = 0);

  /**
   * @brief Destructor. Deallocate memory.
   */
  ~Joystick();

  /***************************************************************************/
  /** Event Filter                                                          **/
  /***************************************************************************/

  /**
   * @brief React on mouse events to move around the joystick.
   * @param ob
   * @param e
   * @return
   */
  bool eventFilter(QObject *ob, QEvent *e);

  /***************************************************************************/
  /** Public Methods                                                        **/
  /***************************************************************************/

  /**
   * @brief Sets the autoReset variable (if the knob should re-center on mouse release).
   * @param autoReset
   */
  void setAutoReset(bool autoReset);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  QLabel *knob_ = nullptr;

  int radius_ = 80;

  int knob_radius_ = 26;
  int knob_width_ = 52;
  int knob_height_ = 52;

  bool mouseReleased_ = true;
  bool autoReset_ = false;

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  /**
   * @brief
   * @param mouseX
   * @param mouseY
   * @return
   */
  QPoint computeKnobPosition(int mouseX, int mouseY);

  void updateJoystickValues(QPoint knobPosition);

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void joystickMoved(double x, double y);
};

} // namespace
