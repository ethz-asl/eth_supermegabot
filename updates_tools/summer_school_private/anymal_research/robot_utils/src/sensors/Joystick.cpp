/*!
 * @file 	Joystick.cpp
 * @author 	Christian Gehring
 * @date 	June 27, 2013
 * @version 1.0
 * @ingroup robot_utils
 */
#include "robot_utils/sensors/Joystick.hpp"

#include <stdio.h>

namespace robot_utils {


Joystick::Joystick() :
		nAxes_(7),
		nButtons_(15),
		joystickAxes_{0.0},
		joystickButtons_{0.0}
{
	for (int i = 0; i < nButtons_; i++) {
		joystickButtonsTimers_[i].pinTime();
	}
	oneClickButtonTimePeriod_ = 0.5;
}

Joystick::~Joystick()
{

}

double Joystick::getAxis(int iAxis) {
	return joystickAxes_[iAxis-1];
}


double Joystick::getButton(int iButton) {
	return joystickButtons_[iButton-1];
}

bool Joystick::getButtonOneClick(int iButton)
{
	if (iButton<1 || iButton>nButtons_) {
		printf("Joystick::Error: Button %d is not available!\n", iButton);
		return false;
	}

	if (joystickButtons_[iButton-1] == 1 && joystickButtonsTimers_[iButton-1].getElapsedTimeSec() > oneClickButtonTimePeriod_) {
		joystickButtonsTimers_[iButton-1].pinTime();
		return true;
	}
	return false;
}


double Joystick::getSagittal() {
	return getAxis(JA_SAGITTAL);
}

double Joystick::getCoronal() {
	return getAxis(JA_CORONAL);
}

double Joystick::getVertical() {
	return getAxis(JA_VERTICAL);
}

double Joystick::getRoll() {
	return getAxis(JA_ROLL);
}

double Joystick::getPitch() {
	return getAxis(JA_PITCH);
}

double Joystick::getYaw() {
	return getAxis(JA_YAW);
}

void Joystick::setSagittal(double value) {
	joystickAxes_[JA_SAGITTAL-1] = value;
}

void Joystick::setCoronal(double value) {
	joystickAxes_[JA_CORONAL-1] = value;
}

void Joystick::setVertical(double value) {
	joystickAxes_[JA_VERTICAL-1] = value;
}

void Joystick::setRoll(double value) {
	joystickAxes_[JA_ROLL-1] = value;
}

void Joystick::setPitch(double value) {
	joystickAxes_[JA_PITCH-1] = value;
}


void Joystick::setYaw(double value) {
	joystickAxes_[JA_YAW-1] = value;
}

double* Joystick::getAxes()
{
	return joystickAxes_;
}

double* Joystick::getButtons()
{
	return joystickButtons_;
}

int Joystick::getNoAxes()
{
	return nAxes_;
}
int Joystick::getNoButtons()
{
	return nButtons_;
}

void Joystick::setAxis(int iAxis, double value)
{
	joystickAxes_[iAxis-1] = value;
}

void Joystick::setButton(int iButton, double value)
{
	joystickButtons_[iButton-1] = value;
}


} /* namespace robotModel */
