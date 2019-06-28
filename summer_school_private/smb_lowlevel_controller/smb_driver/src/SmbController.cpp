/*
 * Controller.cpp
 *
 *  Created on: May 26, 2017
 *      Author: sasutosh
 */

#include<smb_driver/SmbController.h>

#define BOTH_MOTORS 0
#define FIRST_MOTOR 1
#define SECOND_MOTOR 2

namespace smb_driver {

SmbController::SmbController(std::string port, ros::NodeHandle &nh, size_t vecSize, bool sendCommands) :
        sendCommands_(sendCommands),
        vecSize_(vecSize),
        stopAcquisition_(false),
        port_(port),
        nh_(nh)
{
  rpmToRps_ = 2.0 * M_PI / 60.0;

	command_packets_.set_capacity(vecSize_);
//	this->startAcquisition();

  wheelSpeedPub_ = nh_.advertise<std_msgs::Float64MultiArray>("/wheelSpeeds", 1);

  wheelSpeedMsg_.layout.dim.resize(1);
  wheelSpeedMsg_.layout.dim[0].label = "t leftSpeed rightSpeed";
  wheelSpeedMsg_.layout.dim[0].size = 1;
  wheelSpeedMsg_.layout.dim[0].stride = 3;
  wheelSpeedMsg_.data.resize(3);
}

SmbController::~SmbController() {
    stopAcquisition_ = true;
    if (hyambRoboteqSerialThread_.joinable()) {
    	hyambRoboteqSerialThread_.join();
	}
}

void SmbController::startAcquisition() {
    if (!hyambRoboteqSerialThread_.joinable()) {
    	hyambRoboteqSerialThread_ = thread_t(&SmbController::receiveData, this);
    }
    else {
        printf("Error-startAcquisition(): Failed to start the main driver thread! \n");
    }
}

void SmbController::stopAcquisition() {
    stopAcquisition_ = true;
    hyambRoboteqSerialThread_.join();
}

bool SmbController::readWheelSpeeds() {
    bool res = true;
		
    int leftSpeedResult, leftSpeedStatus = -1;
    double leftSpeed;
    leftSpeedStatus = serialDevice->GetValue(_S, 1, leftSpeedResult);
    if(leftSpeedStatus == RQ_SUCCESS) {
        leftSpeed = leftSpeedResult;
        leftSpeed *= rpmToRps_;
    }
    else {
        res = false;
        printf("Failed to get motor1 wheel speed. Status: %i\n", leftSpeedStatus);
    }

    int rightSpeedResult, rightSpeedStatus = -1;
    double rightSpeed;
    rightSpeedStatus = serialDevice->GetValue(_S, 2, rightSpeedResult);
    if(rightSpeedStatus == RQ_SUCCESS) {
        rightSpeed = -rightSpeedResult; //Minus so that positive speed is in forward direction
        rightSpeed *= rpmToRps_;
    }
    else {
        res = false;
        printf("Failed to get motor2 wheel speed. Status: %i\n", rightSpeedStatus);
    }

    if (res) {
      //Publish the wheel speeds for sensor fusion at the rate at which they are queried
      wheelSpeedMsg_.data[0] = ros::Time::now().toSec();
      wheelSpeedMsg_.data[1] = leftSpeed;
      wheelSpeedMsg_.data[2] = rightSpeed;
      wheelSpeedPub_.publish(wheelSpeedMsg_);
    }

    acquireMutex(dataMutex_, 0);

    if (leftSpeedStatus == RQ_SUCCESS)
        leftMotorSpeed_ = leftSpeed;
    if(rightSpeedStatus == RQ_SUCCESS)
        rightMotorSpeed_ = rightSpeed;

    releaseMutex(dataMutex_);

    return res;
}

bool SmbController::readBatteryVoltage() {
    bool res = true;

    int batteryVoltageResult, batteryVoltageStatus = -1;
    double batteryVoltage;
    batteryVoltageStatus = serialDevice->GetValue(_V, 2, batteryVoltageResult);
    if(batteryVoltageStatus == RQ_SUCCESS)
        batteryVoltage = batteryVoltageResult/10.0;
    else {
        res = false;
        printf("Failed to get battery voltage. Status: %i\n", batteryVoltageStatus);
    }

    //Write the measured values to the protected memory
    //todo No timeout is used here on the controller side. Is that good?
    acquireMutex(dataMutex_, 0);

    if(batteryVoltageStatus == RQ_SUCCESS)
        batteryVoltage_ = batteryVoltage;

    releaseMutex(dataMutex_);

    return res;
}

bool SmbController::setDesiredCommands()
{
    bool res = true;

	acquireMutex(desiredCmdMutex_, 0);
	switch(mode_)
	{
	case CLOSED_LOOP_SPEED:
		if (!setVelocityImpl(des_velocity_motor1_, 1)) res = false;
        if (!setVelocityImpl(des_velocity_motor2_, 2)) res = false;
		break;
	case TORQUE:
        if (!setTorqueImpl(des_tau_motor1_, 1)) res = false;
        if (!setTorqueImpl(des_tau_motor2_, 2)) res = false;
		break;
	case OPEN_LOOP:
        if (!setMotorPowerImpl(des_pow_motor1_, 1)) res = false;
        if (!setMotorPowerImpl(des_pow_motor2_, 2)) res = false;
		break;
	default:
		printf("This mode has not been implemented. \n");
		break;
	}
	releaseMutex(desiredCmdMutex_);

	return res;
}

bool SmbController::connect() {
	isReady_ = false; //This is only set true once we get a positive handshake with the device

	serialDevice = std::make_shared<RoboteqDevice>();

	int status = serialDevice->Connect(port_);
	if(status != RQ_SUCCESS) {
		printf("Error connecting to motor controller. Status: %i. \n", status);
		return false;
	}

	//Query the command mode to ensure a proper connection
	int reply = -1;
	int result = 0;
	int nTrials = 3;
	while(nTrials > 0) {
		nTrials--;
		reply = serialDevice->GetConfig(_MMOD, 1, result);
		if(reply == RQ_SUCCESS) {
			acquireMutex(dataMutex_, 0);
			mode_ = (HYAMBModes)result;
			releaseMutex(dataMutex_);
			isReady_ = true;

			printf("Motor controller connected in mode %i\n", mode_);
		}
		return true;
	}

	printf("Querying the controller mode return error code %i. Failed to connect.\n", reply);

	return false;
}

void SmbController::receiveData(void *context) {
	SmbController *instance = (SmbController *) context;

	auto cycleCountStartTime_ = std::chrono::high_resolution_clock::now();
	int cycleCount_ = 0;
	instance->cycleCountPeriod_Us_ = 5000000.0;
  instance->t_lastSuccessfulCycle_ = cycleCountStartTime_;

	while (!instance->stopAcquisition_){
    auto startLoop_ = std::chrono::high_resolution_clock::now();
    bool res = true;

    if (!instance->isConnected()) {
      if(!instance->connect()) {
        sleepms(1000); //Sleep for 1 sec and try again
        continue;
      }
    }

		//Issue the commands
		if (instance->sendCommands_) {
      if (!instance->setDesiredCommands())
        res = false;
    }

		//Send additional commands if desired
		//Currently just setting the command mode is supported
		while(instance->requestAvailable_)
		{
			CommandPacket cmd;
			cmd.cmdType = INVALID;
			acquireMutex(instance->cmdBufferMutex_, 0);
			if(instance->command_packets_.empty())
			{
				instance->requestAvailable_ = false;
			}
			else
			{
				cmd = instance->command_packets_.back();
				instance->command_packets_.pop_back();
				if(instance->command_packets_.empty())
				{
					instance->requestAvailable_ = false;
				}
			}
			releaseMutex(instance->cmdBufferMutex_);

			switch(cmd.cmdType)
			{
			case Mode:
				if (!instance->setModeImpl((int)cmd.value, cmd.motor)) {
				    res = false;
				}
				break;
			case INVALID:
				break;
			default:
				break;
			}
		}

		//Read the desired inputs (wheel speeds and battery voltage)
    if (!instance->readWheelSpeeds())
      res = false;

		if ((instance->t_lastVoltageUpdate_ - startLoop_).count() > instance->batteryVoltageUpdateInterval_Us_) {
      if (!instance->readBatteryVoltage())
        res = false;
      else {
        double batteryVoltage;
        instance->getBatteryVoltage(batteryVoltage, 500);
        if (batteryVoltage < instance->lowBatteryVoltageWarningLevel_)
          printf("[SmbController] WARNING: Low battery voltage. batteryVoltage=%f V", batteryVoltage);
      }
    }

    auto endLoop_ = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> elapsedTime_ = endLoop_ - startLoop_; // fractional duration: no duration_cast needed
    if(instance->min_cycle_time_Us_ > elapsedTime_.count())
      usleep(instance->min_cycle_time_Us_ - elapsedTime_.count());
//			else
//				printf("mb driver over-ran its cycle time by %f sec\n", -1e-6 * elapsedTime_.count());
//printf("mb driver cycle took %f microsec\n",elapsedTime_.count());
    ++cycleCount_;

    //Periodically print the cycle time for debugging
    std::chrono::duration<double, std::micro> cycleCountTime = endLoop_ - cycleCountStartTime_;
    if (cycleCountTime.count() > instance->cycleCountPeriod_Us_) {
//      double avgCycleTime = cycleCountTime.count() * 1e-6 / cycleCount_;
//      printf("Motor controller io thread average cycle time: %f\n", avgCycleTime);

      //Reset the cycle counter
      cycleCountStartTime_ = endLoop_;
      cycleCount_ = 0;
    }


    if (res) {
        instance->t_lastSuccessfulCycle_ = endLoop_;
    }
    else if ((endLoop_ - instance->t_lastSuccessfulCycle_).count() > instance->communicationDropoutTime_Us_) {
        //Try to reconnect to the motor controller
        instance->isReady_ = false;
        printf("[SmbController] Comunication with motor controller has been lost. Trying to reconnect...\n");
    }
	}
}

bool SmbController::getWheelSpeeds(double& leftSpeed, double& rightSpeed, int timeoutUs) {
	if (acquireMutex(dataMutex_, timeoutUs)) {
		leftSpeed = leftMotorSpeed_;
		rightSpeed = rightMotorSpeed_;
		releaseMutex(dataMutex_);		

		return true;
	}
	return false;
}

bool SmbController::getBatteryVoltage(double& batteryVoltage, int timeoutUs) {
	if (acquireMutex(dataMutex_, timeoutUs)) {
		batteryVoltage = batteryVoltage_;
		releaseMutex(dataMutex_);
		return true;
	}
	return false;
}

double SmbController::getVelocity(int motor)
{
	double velocity;
	acquireMutex(dataMutex_, 0);
	if(motor == 1)
		velocity = leftMotorSpeed_;
	else
		velocity = rightMotorSpeed_;
	releaseMutex(dataMutex_);
	return velocity;
}

HYAMBModes SmbController::getMode()
{
	return mode_; // assuming mode does not change on the fly
}

void SmbController::setMode(HYAMBModes mod, int motor)
{
	CommandPacket cmd;
	cmd.cmdType = Mode;
	cmd.value = mod;
	cmd.motor = motor;

	acquireMutex(cmdBufferMutex_, 0);
	command_packets_.push_back(cmd);
	releaseMutex(cmdBufferMutex_);
	requestAvailable_ = true;
}

void SmbController::setMode(HYAMBModes mod)
{
	setMode(mod, BOTH_MOTORS);
}

//vel should be rad/sec
void SmbController::setVelocity(double vel, int motor)
{
	HYAMBModes mod = getMode();
	if(mod != CLOSED_LOOP_SPEED)
	{
		printf("Error SmbController::setVelocity: This command is not allowed in the present mode. \n");
		return;
	}
	if(vel > 1000 || vel < -1000)
	{
		printf("Error SmbController::setVelocity: Invalid velocity value. Enter between -1000 to 1000 \n");
		return;
	}

	acquireMutex(desiredCmdMutex_, 0);
	if(motor == 1)
	{
		des_velocity_motor1_ = vel * config_rps_to_cmd_;
	}
	else if(motor == 2)
	{
		des_velocity_motor2_ = vel * config_rps_to_cmd_;
	}
	else
	{
		des_velocity_motor1_ = vel;
		des_velocity_motor2_ = vel;
	}
	releaseMutex(desiredCmdMutex_);
}

void SmbController::setVelocity(double vel)
{
	setVelocity(vel, BOTH_MOTORS);
}

//This is actually setting the desired motor current
//Cmd = I_des / I_max * 1000
void SmbController::setTorque(double tau, int motor)
{
	HYAMBModes mod = getMode();
	if(mod != TORQUE)
	{
//		printf("Error SmbController::setTorque: This command is not allowed in the present mode. \n");
		return;
	}
	if(tau > 1000 || tau < -1000)
	{
		printf("Error SmbController::setTorque: Invalid torque value of %f. Enter between -1000 to 1000 \n", tau);
		return;
	}

	acquireMutex(desiredCmdMutex_, 0);
	if(motor == 1)
	{
		des_tau_motor1_ = tau;
	}
	else if(motor == 2)
	{
		des_tau_motor2_ = tau;
	}
	else
	{
		des_tau_motor1_ = tau;
		des_tau_motor2_ = tau;
	}
	releaseMutex(desiredCmdMutex_);
}

void SmbController::setTorque(double tau)
{
	setTorque(tau, BOTH_MOTORS);
}

void SmbController::setMotorPower(double pow, int motor)
{
//	CommandPacket cmd;
//	cmd.cmdType = Motor_Power;
//	cmd.value = pow;
//	cmd.motor = motor;
//
//	acquireMutex(cmdBufferMutex_, -1);
//	command_packets_.push_back(cmd);
//	releaseMutex(cmdBufferMutex_);
//	requestAvailable_ = true;

	HYAMBModes mod = getMode();
	if(mod != OPEN_LOOP)
	{
		printf("Error SmbController::setMotorPower: This command is not allowed in the present mode. Current mode is %d \n", mod);
		return;
	}
	if(pow > 1000 || pow < -1000)
	{
		printf("Error SmbController::setMotorPower: Invalid Motor Power value. Enter between -1000 to 1000 \n");
		return;
	}

	acquireMutex(desiredCmdMutex_, 0);
	if(motor == 1)
	{
		des_pow_motor1_ = pow;
	}
	else if(motor == 2)
	{
		des_pow_motor2_ = pow;
	}
	else
	{
		des_pow_motor1_ = pow;
		des_pow_motor2_ = pow;
	}
	releaseMutex(desiredCmdMutex_);
}

void SmbController::setMotorPower(double pow)
{
	setMotorPower(pow, BOTH_MOTORS);
}

void SmbController::setFreeze() {
  HYAMBModes mode = getMode();
  switch (mode) {
    case OPEN_LOOP:
      acquireMutex(desiredCmdMutex_, 0);
      des_pow_motor1_ = 0.0;
      des_pow_motor2_ = 0.0;
      releaseMutex(desiredCmdMutex_);
      break;
    case TORQUE:
      acquireMutex(desiredCmdMutex_, 0);
      des_tau_motor1_ = 0.0;
      des_tau_motor2_ = 0.0;
      releaseMutex(desiredCmdMutex_);
      break;
    case CLOSED_LOOP_SPEED:
      acquireMutex(desiredCmdMutex_, 0);
      des_velocity_motor1_ = 0.0;
      des_velocity_motor2_ = 0.0;
      releaseMutex(desiredCmdMutex_);
      break;
    default:
      printf("Unsupported operating mode when setFreeze was called?? mode=%d\n", mode);
      break;
  }
}

bool SmbController::setModeImpl(int mode)
{
	return setModeImpl(mode, BOTH_MOTORS);
}

bool SmbController::setModeImpl(int mode, int motor)
{
	int status = -1;
	HYAMBModes existingMode = getMode();
	if(mode == CLOSED_LOOP_POSITION_RELATIVE || mode == CLOSED_LOOP_COUNT_POSITION ||
			mode == CLOSED_LOOP_POSITION_TRACKING || mode == CLOSED_LOOP_SPEED_POSITION)
	{
		printf("[SmbController] Error: An unsupported command mode has been requested. Please try a valid mode. \n");
		return true; //We can't change the mode, but this does not indicate a failure in the device connection
	}
	// change the first motor mode
	if((status = serialDevice->SetConfig(_MMOD, 1, mode)) != RQ_SUCCESS)
	{
		printf("Error: Failed to set mode.\n");
		return false;
	}
	if((status = serialDevice->SetConfig(_MMOD, 2, mode)) != RQ_SUCCESS)
	{
        printf("Error: Failed to the set mode of the right motor. Reverting to the previous mode.\n");
	    //Return to the previous mode on the first motor
		if((status = serialDevice->SetConfig(_MMOD, 1, existingMode)) != RQ_SUCCESS)
			printf("Error: Motor 1 and Motor 2 have been set two different modes. Please fix this. \n");

		return false;
	}

	acquireMutex(dataMutex_, 0);
	this->mode_ = (HYAMBModes)mode;
	releaseMutex(dataMutex_);

	return true;
}

bool SmbController::setVelocityImpl(double vel, int motor)
{
	int res = sendGoCommand(vel, motor);
	if(res != RQ_SUCCESS) {
        printf("Error SmbController::setVelocityImpl: Set Commands failed for motor %i, returning %i \n", motor, res);
        return false;
    }

    return true;
}

bool SmbController::setVelocityImpl(double vel)
{
	return setVelocityImpl(vel, BOTH_MOTORS);
}

bool SmbController::setTorqueImpl(double tau, int motor)
{
	int res = sendGoCommand(tau, motor);
	if(res != RQ_SUCCESS) {
        printf("Error SmbController::setTorqueImpl: Set Commands failed for motor %i, returning %i \n", motor, res);
        return false;
    }

    return true;
}

bool SmbController::setTorqueImpl(double tau)
{
	return setTorqueImpl(tau, BOTH_MOTORS);
}

bool SmbController::setMotorPowerImpl(double pow, int motor)
{
	int res = sendGoCommand(pow, motor);
	if(res != RQ_SUCCESS) {
		printf("Error SmbController::setMotorPowerImpl: Set Commands failed for motor %i, returning %i \n", motor,res);
        return false;
    }

    return true;
}

bool SmbController::setMotorPowerImpl(double pow)
{
	return setMotorPowerImpl(pow, BOTH_MOTORS);
}

int SmbController::sendGoCommand(double value, int motor)
{
	int status =-1;
	if(motor == BOTH_MOTORS)
	{
		if((status = serialDevice->SetCommand(_GO, 1, value)) != RQ_SUCCESS)
		{
			return RQ_SET_COMMAND_FAILED;
		}
		if((status = serialDevice->SetCommand(_GO, 2, -value)) != RQ_SUCCESS)
		{
			return RQ_SET_COMMAND_FAILED;
		}
	}
	else
	{
		if(motor == 2)
		{
			value = -value; //Make sure +vel is in +x robot direction!
		}
		if((status = serialDevice->SetCommand(_GO, motor, value)) != RQ_SUCCESS)
		{
			return RQ_SET_COMMAND_FAILED;
		}
	}
	return RQ_SUCCESS;
}

void SmbController::eStop()
{
	int status = -1;
	if((status = serialDevice->SetCommand(_EX)) != RQ_SUCCESS)
	{
		printf("Error: Could not activate ESTOP!!!!!");
	}
}

void SmbController::eStopRelease()
{
	int status = -1;
	if((status = serialDevice->SetCommand(_MG)) != RQ_SUCCESS)
	{
		printf("Error: Could not release ESTOP!!!!!");
	}
}

bool SmbController::isConnected()
{
	return isReady_;
}

} //namespace smb_driver
