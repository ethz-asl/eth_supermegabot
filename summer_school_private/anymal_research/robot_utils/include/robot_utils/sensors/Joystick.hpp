/*!
 * @file 	Joystick.hpp
 * @author 	Christian Gehring
 * @date 	June 27, 2013
 * @version 1.0
 * @ingroup robot_utils
 */

#pragma once

#include <std_utils/timers/ChronoTimer.hpp>

namespace robot_utils {

class Joystick {
public:
	enum JoyAxes {
		JA_SAGITTAL=2,
		JA_CORONAL=1,
		JA_VERTICAL=5,
		JA_ROLL=3,
		JA_PITCH=6,
		JA_YAW=4,
	};

	Joystick();
	virtual ~Joystick();

	/*! Gets the joystick axis by index  {1, ..., 7}
	 * @param iAxis index of the axis
	 * @return value
	 */
	double getAxis(int iAxis);

	/*! Gets the joystick button by button number {1, ..., 15}
	 * @param iButton index of the button {1, ..., 15}
	 * @return value
	 */
	double getButton(int iButton);

	bool getButtonOneClick(int iButton);

	double getSagittal();

	double getCoronal();

	double getVertical();

	double getRoll();

	double getPitch();

	double getYaw();

	void setSagittal(double value);

	void setCoronal(double value);

	void setVertical(double value);

	void setRoll(double value);

	void setPitch(double value);

	void setYaw(double value);

	double* getAxes();

	double* getButtons();

	int getNoAxes();
	int getNoButtons();

	void setAxis(int iAxis, double value);

	void setButton(int iButton, double value);
protected:
	//! number of axes
	const int nAxes_;
	//! number of buttons
	const int nButtons_;

	//! joystick axes
	double joystickAxes_[7];

	//! joystick buttons
	double joystickButtons_[15];

	//! button timers
	std_utils::SteadyClockTimer joystickButtonsTimers_[15];

	//! time period of a single clicks
	double oneClickButtonTimePeriod_;

};

} /* namespace robotModel */
