#include <iostream>
#include <stdio.h>
#include <string.h>
#include "smb_driver/RoboteqDevice.h"
#include "smb_driver/ErrorCodes.h"
#include "smb_driver/Constants.h"
#include "smb_driver/auxiliaries/interProcessCommunication.h"
#include<color.hpp>

using namespace std; // not a good practice

int main(int argc, char *argv[])
{
	string response = "";
	RoboteqDevice device;

	int status = device.Connect("/dev/ttySMB");

	if(status != RQ_SUCCESS)
	{
		cout<< red_ <<"Error connecting to device: "<<status<<"."<< def_ <<endl;
		return 1;
	}

	int result;

	//*****************************************************************
	// Battery Information
	// Roboteq manual: result = Volts * 10 for internal and battery volts.
	// 							Milivolts for 5V output
	//*****************************************************************
	std::cout << green_ << "***Battery Status***" << def_ << std::endl;
	cout<< blue_ << "\t- Internal Volts... ";
	if((status = device.GetValue(_V, 1, result)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status << def_ << endl;
	else
		cout<< green_ << result/10.0 << "V" << def_ << endl;

	cout<< blue_ << "\t- Battery Volts... ";
	if((status = device.GetValue(_V, 2, result)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status << def_ << endl;
	else
		cout<< green_ << result/10.0 << "V" << def_ << endl;

	cout<< blue_ << "\t- 5V Output... ";
	if((status = device.GetValue(_V, 3, result)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status << def_ << endl;
	else
		cout<< green_ << result/1000.0 << "V" << def_ << endl;

	//*****************************************************************
	// Basic Configurations : Mode
	//*****************************************************************
	std::cout << green_ << "***Config***" << def_ << std::endl;
	cout<< blue_ << "\t- GetConfig(_MMOD, 2)...";
	if((status = device.GetConfig(_MMOD, 2, result)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status <<endl;
	else
		cout<< green_ << "returned --> "<< result <<endl;

	cout<< blue_ << "\t- GetConfig(_MMOD, 1)...";
	if((status = device.GetConfig(_MMOD, 1, result)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status <<endl;
	else
		cout<< green_ << "returned --> "<< result <<endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	cout<< blue_ << "\t- SetConfig(_MMOD, 1, 0)...";
	if((status = device.SetConfig(_MMOD, 1, 0)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status << def_ << endl;
	else
		cout<< green_ << "succeeded."<< def_ << endl;

	cout<< blue_ << "\t- SetConfig(_MMOD, 2, 0))...";
	if((status = device.SetConfig(_MMOD, 2, 0)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status << def_ << endl;
	else
		cout<< green_ << "succeeded."<< def_ << endl;

	//*****************************************************************
	// Basic Commands
	//*****************************************************************
	std::cout << green_ << "***Basic Commands***" << def_ << std::endl;
	cout<< blue_ << "\t- SetCommand(_GO, 1, 100)...";
	if((status = device.SetCommand(_GO, 1, 100)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status << def_ << endl;
	else
		cout<< green_ << "succeeded."<< def_ << endl;

	cout<< blue_ << "\t- SetCommand(_GO, 2, -100)...";
	if((status = device.SetCommand(_GO, 2, -100)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status << def_ << endl;
	else
		cout<< green_ << "succeeded."<< def_ << endl;
	//Wait 10 ms before sending another command to device
	sleepms(1000);

	cout<< blue_ << "\t- SetCommand(_GO, 1, 0)...";
	if((status = device.SetCommand(_GO, 1, 0)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status << def_ << endl;
	else
		cout<< green_ << "succeeded."<< def_ << endl;

	cout<< blue_ << "\t- SetCommand(_GO, 2, 0)...";
	if((status = device.SetCommand(_GO, 2, 0)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<< status << def_ << endl;
	else
		cout<< green_ << "succeeded."<< def_ << endl;

	//Wait 10 ms before sending another command to device
	sleepms(10);

	device.Disconnect();
	return 0;
}
