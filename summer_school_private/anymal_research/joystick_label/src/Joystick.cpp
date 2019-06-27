/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "joystick_label/Joystick.h"

namespace joystick_label {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

Joystick::Joystick(QWidget *parent) : QLabel(parent) {

  this->installEventFilter(this);

  this->setToolTip("<p>Shift+Mouse: snap to lines</p>"
                       "<p>Double click to reset</p>");

  this->setText("");
  this->setScaledContents(true);
  this->setPixmap(
      QPixmap(QString::fromStdString(ros::package::getPath("joystick_label")
                                         .append("/resource/background.png"))));
  this->setFixedSize(160, 160);

  knob_ = new QLabel();
  knob_->setParent(this);
  knob_->setGeometry(radius_ - knob_radius_, radius_ - knob_radius_,
                     knob_width_, knob_height_);
  knob_->setScaledContents(true);
  knob_->setAttribute(Qt::WA_TranslucentBackground, true);
  knob_->setPixmap(
      QPixmap(QString::fromStdString(ros::package::getPath("joystick_label")
                                         .append("/resource/knob.png"))));
}

Joystick::~Joystick() {
  if (knob_ != nullptr) {
    delete knob_;
    knob_ = nullptr;
  }
}

/*****************************************************************************/
/** Event Filter                                                            **/
/*****************************************************************************/

bool Joystick::eventFilter(QObject *ob, QEvent *e) {
  // mouse double click -> reset knob to center
  if (e->type() == QEvent::MouseButtonDblClick) {
    mouseReleased_ = true;
    knob_->move(computeKnobPosition(radius_, radius_));
    updateJoystickValues(knob_->pos());

    return QLabel::eventFilter(ob, e);
  }

  if (e->type() == QEvent::MouseButtonPress) {
    QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(e);
    mouseReleased_ = false;

    if (Qt::ShiftModifier == QApplication::queryKeyboardModifiers()) {
      if (abs(mouseEvent->x() - radius_) > abs(mouseEvent->y() - radius_)) {
        knob_->move(computeKnobPosition(mouseEvent->x(), radius_));
      } else {
        knob_->move(computeKnobPosition(radius_, mouseEvent->y()));
      }
    } else {
      knob_->move(computeKnobPosition(mouseEvent->x(), mouseEvent->y()));
    }
    updateJoystickValues(knob_->pos());
  }

  if (e->type() == QEvent::MouseButtonRelease) {
    mouseReleased_ = true;
    // If autoReset_ is true, return the knob to center on mouse button release
    if (autoReset_){
      knob_->move(computeKnobPosition(radius_, radius_));
      updateJoystickValues(knob_->pos());
    }
  }

  if (e->type() == QEvent::MouseMove && !mouseReleased_) {
    QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(e);

    if (Qt::ShiftModifier == QApplication::queryKeyboardModifiers()) {
      if (abs(mouseEvent->x() - radius_) > abs(mouseEvent->y() - radius_)) {
        knob_->move(computeKnobPosition(mouseEvent->x(), radius_));
      } else {
        knob_->move(computeKnobPosition(radius_, mouseEvent->y()));
      }
    } else {
      knob_->move(computeKnobPosition(mouseEvent->x(), mouseEvent->y()));
    }
    updateJoystickValues(knob_->pos());
  }

  return QLabel::eventFilter(ob, e);
}

/***************************************************************************/
/** Public Methods                                                        **/
/***************************************************************************/

/**
 * @brief Sets the autoReset variable (if the knob should reset on mouse release).
 * @param autoReset
 */
void Joystick::setAutoReset(bool autoReset){
  autoReset_ = autoReset;
  if (autoReset_){
    // Centers the knob
    knob_->move(computeKnobPosition(radius_, radius_));
    updateJoystickValues(knob_->pos());
  }
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

QPoint Joystick::computeKnobPosition(int mouseX, int mouseY) {
  if (sqrt(pow(mouseX - radius_, 2.0) + pow(mouseY - radius_, 2.0)) >
      radius_ - knob_radius_) {
    double v_x = mouseX - radius_;
    double v_y = mouseY - radius_;
    double x = v_x / sqrt(pow(v_x, 2.0) + pow(v_y, 2.0));
    double y = v_y / sqrt(pow(v_x, 2.0) + pow(v_y, 2.0));
    x *= (double)(radius_ - knob_radius_);
    y *= (double)(radius_ - knob_radius_);
    return QPoint((int)(radius_ + x - knob_radius_),
                  (int)(radius_ + y - knob_radius_));
  } else {
    return QPoint(mouseX - knob_radius_, mouseY - knob_radius_);
  }
}

void Joystick::updateJoystickValues(QPoint knobPosition) {
  double x = knobPosition.x() + knob_radius_ - radius_;
  double y = knobPosition.y() + knob_radius_ - radius_;
  double span = radius_ - knob_radius_;
  x /= -span;
  y /= -span;
  x = (x == 0 ? -x : x);
  y = (y == 0 ? -y : y);

  emit joystickMoved(x, y);
}

} // namespace
