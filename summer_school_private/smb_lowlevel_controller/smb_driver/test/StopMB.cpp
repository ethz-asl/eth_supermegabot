/*
 * StopMB.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: sasutosh
 */

#include <iostream>
#include <stdio.h>
#include <string.h>

#include "smb_driver/RoboteqDevice.h"
#include "smb_driver/ErrorCodes.h"
#include "smb_driver/Constants.h"
#include "smb_driver/auxiliaries/interProcessCommunication.h"

#include<color.hpp>

using namespace std;

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

	cout<< blue_ << "- SetCommand(_GO, 1, 0)...";
	if((status = device.SetCommand(_GO, 1, 0)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<<status<< def_ << endl;
	else
		cout<< green_ << "succeeded."<< def_ << endl;

	cout<< blue_ << "- SetCommand(_GO, 2, 0)...";
	if((status = device.SetCommand(_GO, 2, 0)) != RQ_SUCCESS)
		cout<< red_ << "failed --> "<<status<< def_ << endl;
	else
		cout<< green_ << "succeeded."<< def_ << endl;
		
	sleepms(10);

	device.Disconnect();
	return 0;
}


