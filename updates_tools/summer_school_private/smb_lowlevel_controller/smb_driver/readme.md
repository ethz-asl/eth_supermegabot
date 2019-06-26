# HYA Mobile Base Low Level Control

### TODO
- Implment realtime serial read
- More test cases
- Battery status
- Estop better implementation

## How to use

### Available modes
```c++
enum HYAMBModes{
OPEN_LOOP = 0,
CLOSED_LOOP_SPEED,
CLOSED_LOOP_POSITION_RELATIVE,				// not impemented
CLOSED_LOOP_COUNT_POSITION,					// not impemented
CLOSED_LOOP_POSITION_TRACKING,				// not impemented
TORQUE,
CLOSED_LOOP_SPEED_POSITION					// not impemented
};
```
### Exposed functionalities
```c++
	double getVelocity();						// returns the average of two motors
	double getVelocity(int motor);				// returns the motor velocity in RPM
	void setVelocity(double vel);				// sets the velocity for closed loop speed mode for both motors
	void setVelocity(double vel, int motor);	// sets the velocity for closed loop speed mode for the motor
	void setTorque(double tau);					// sets the velocity for closed loop torque mode for both motors
	void setTorque(double tau, int motor);		// sets the velocity for closed loop torque mode for thr motor
	void setMotorPower(double pow);				// sets the motor power for open loop mode for both motors
	void setMotorPower(double pow, int motor);	// sets the motor power for open loop mode for the motor
	HYAMBModes getMode();						// returns the current mode
	void setMode(HYAMBModes mod);				// sets the mode for both motors
	void setMode(HYAMBModes mod, int motor);	// sets the mode for the specified motor
	void eStop();								// activate Emergency stop
	void eStopRelease();						// Releasre emergency stop
	void stopAcquisition();						// clean up after the task is done
```

### Switching between Xeno and simulation
Change the the Cmakelists.txt Line number 5 
```bash
set(HYA_MB_USE_XENO ON) # ON - XENO Mode; OFF: non-xeno mode
```
### Test Cases
To test the low level serial communication:
```bash
rosrun hya_mb_lowlvl_ctrl testSerial
```

To test the high level cotroller communication, please run the following command. It runs different tests for xeno and non-xeno mode. Make sure to test both cases.
```bash
rosrun hya_mb_lowlvl_ctrl testController
```

## Tips

### Stop Mobile Base
In case something goes wrong and the wheel keep turning even after the program is over, please run the following command to stop the MB

```bash
rosrun hya_mb_lowlvl_ctrl stopMB
```

### Catching mode switches
