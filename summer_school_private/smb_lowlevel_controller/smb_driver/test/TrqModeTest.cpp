#include <iostream>
#include <stdio.h>
#include <string.h>
#include "smb_driver/SmbController.h"
//#include "RoboteqDevice.h"
//#include "ErrorCodes.h"
//#include "Constants.h"
//#include "auxiliaries/interProcessCommunication.h"
#include<color.hpp>
#include <ros/ros.h>

int timeoutUs = 500;
double batteryVoltage, leftWheelSpeed, rightWheelSpeed;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "TrqModeTest");
	ros::NodeHandle nh;

	std::string response = "";
	smb_driver::SmbController mbSmbController("/dev/ttySMB");
//	RoboteqDevice device;

	while (ros::ok()) {
		if(mbSmbController.connect())
			break;
		else
			sleepms(1000); //Sleep for 1 sec and try again
	}

	if (!mbSmbController.isConnected()) {
		std::cout << "SmbController is not connected. Exiting." << std::endl;
		return 0;
	}

	double t_init = ros::Time::now().toSec();

	while (ros::ok()) {
		double t0 = ros::Time::now().toSec();
		//Get the inputs
		mbSmbController.updateAll();
		mbSmbController.getWheelSpeeds(leftWheelSpeed, rightWheelSpeed, timeoutUs);
		mbSmbController.getBatteryVoltage(batteryVoltage, timeoutUs);
		printf("left_speed: %f, right_speed: %f, battery_volt: %f\n",
				leftWheelSpeed, rightWheelSpeed, batteryVoltage);
		double t1 = ros::Time::now().toSec();
		std::cout << "Reading the inputs took " << t1-t0 << " sec" << std::endl;

		//Send the commands
		t0 = ros::Time::now().toSec();
		if (fmod(ros::Time::now().toSec() - t_init, 6.0) > 4.0) {
			mbSmbController.setTorque(1.0, 1);
			mbSmbController.setTorque(1.0, 2);
		}
		else if (fmod(ros::Time::now().toSec() - t_init, 6.0) > 2.0) {
			mbSmbController.setTorque(-1.0, 1);
			mbSmbController.setTorque(0.0, 2);
		}
		else {
			mbSmbController.setTorque(0.0, 1);
			mbSmbController.setTorque(0.0, 2);
		}
		mbSmbController.setDesiredCommands();

		t1 = ros::Time::now().toSec();
		std::cout << "Setting the commands took " << t1-t0 << " sec" << std::endl;

		ros::Duration(0.01).sleep();
	}

//	//*****************************************************************
//	// Battery Information
//	// Roboteq manual: result = Volts * 10 for internal and battery volts.
//	// 							Milivolts for 5V output
//	//*****************************************************************
//	std::cout << green_ << "***Battery Status***" << def_ << std::endl;
//	std::cout<< blue_ << "\t- Internal Volts... ";
//	if((status = device.GetValue(_V, 1, result)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status << def_ << std::endl;
//	else
//		std::cout<< green_ << result/10.0 << "V" << def_ << std::endl;
//
//	std::cout<< blue_ << "\t- Battery Volts... ";
//	if((status = device.GetValue(_V, 2, result)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status << def_ << std::endl;
//	else
//		std::cout<< green_ << result/10.0 << "V" << def_ << std::endl;
//
//	std::cout<< blue_ << "\t- 5V Output... ";
//	if((status = device.GetValue(_V, 3, result)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status << def_ << std::endl;
//	else
//		std::cout<< green_ << result/1000.0 << "V" << def_ << std::endl;

//	//*****************************************************************
//	// Basic Configurations : Mode
//	//*****************************************************************
//	std::cout << green_ << "***Config***" << def_ << std::endl;
//	std::cout<< blue_ << "\t- GetConfig(_MMOD, 2)...";
//	if((status = device.GetConfig(_MMOD, 2, result)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status <<std::endl;
//	else
//		std::cout<< green_ << "returned --> "<< result <<std::endl;
//
//	std::cout<< blue_ << "\t- GetConfig(_MMOD, 1)...";
//	if((status = device.GetConfig(_MMOD, 1, result)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status <<std::endl;
//	else
//		std::cout<< green_ << "returned --> "<< result <<std::endl;
//
//	//Wait 10 ms before sending another command to device
//	sleepms(10);
//
//	std::cout<< blue_ << "\t- SetConfig(_MMOD, 1, 0)...";
//	if((status = device.SetConfig(_MMOD, 1, 0)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status << def_ << std::endl;
//	else
//		std::cout<< green_ << "succeeded."<< def_ << std::endl;
//
//	std::cout<< blue_ << "\t- SetConfig(_MMOD, 2, 0))...";
//	if((status = device.SetConfig(_MMOD, 2, 0)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status << def_ << std::endl;
//	else
//		std::cout<< green_ << "succeeded."<< def_ << std::endl;
//
//	//*****************************************************************
//	// Basic Commands
//	//*****************************************************************
//	std::cout << green_ << "***Basic Commands***" << def_ << std::endl;
//	std::cout<< blue_ << "\t- SetCommand(_GO, 1, 100)...";
//	if((status = device.SetCommand(_GO, 1, 100)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status << def_ << std::endl;
//	else
//		std::cout<< green_ << "succeeded."<< def_ << std::endl;
//
//	std::cout<< blue_ << "\t- SetCommand(_GO, 2, -100)...";
//	if((status = device.SetCommand(_GO, 2, -100)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status << def_ << std::endl;
//	else
//		std::cout<< green_ << "succeeded."<< def_ << std::endl;
//	//Wait 10 ms before sending another command to device
//	sleepms(1000);
//
//	std::cout<< blue_ << "\t- SetCommand(_GO, 1, 0)...";
//	if((status = device.SetCommand(_GO, 1, 0)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status << def_ << std::endl;
//	else
//		std::cout<< green_ << "succeeded."<< def_ << std::endl;
//
//	std::cout<< blue_ << "\t- SetCommand(_GO, 2, 0)...";
//	if((status = device.SetCommand(_GO, 2, 0)) != RQ_SUCCESS)
//		std::cout<< red_ << "failed --> "<< status << def_ << std::endl;
//	else
//		std::cout<< green_ << "succeeded."<< def_ << std::endl;
//
//	//Wait 10 ms before sending another command to device
//	sleepms(10);
//
//	device.Disconnect();
	return 0;
}
