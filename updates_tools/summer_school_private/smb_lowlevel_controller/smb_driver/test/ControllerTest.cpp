/*
 * ControllerTest.cpp
 *
 *  Created on: May 26, 2017
 *      Author: sasutosh
 */

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <memory>
#include <smb_driver/RoboteqBase.h>
#include <smb_driver/SmbController.h>
#include "smb_driver/RoboteqDevice.h"
#include "smb_driver/ErrorCodes.h"
#include "smb_driver/Constants.h"
#include "smb_driver/auxiliaries/interProcessCommunication.h"
#include <color.hpp>

void testMotorPower(smb_driver::SmbController& controller)
{
	std::cout << green_ <<"***Testing setMotorPower()***" << def_ << std::endl;
	if(controller.isConnected())
	{
		std::cout << blue_ << "\tSetting open_loop mode" << def_ << std::endl;
		controller.setMode(smb_driver::OPEN_LOOP);
		sleepms(1000);
		std::cout << blue_ << "\tSetting motorPower to 100" << def_ << std::endl;
		controller.setMotorPower(100);
		sleepms(3000);
		std::cout << blue_ << "\tSetting motor 1 motorPower to 200" << def_ << std::endl;
		controller.setMotorPower(200, 1);
		sleepms(3000);
		std::cout << blue_ << "\tSetting motorPower to 0" << def_ << std::endl;
		controller.setMotorPower(0);
		sleepms(1000);
	}
	else
	{
		std::cout << red_ << "\tError: SmbController is not connected/ready. "<< def_ << std::endl;
	}
}

void testEstop(smb_driver::SmbController& controller)
{
	std::cout << green_ << "***Testing eStop()***" << def_ << std::endl;
	if(controller.isConnected())
	{
		controller.setMotorPower(100);
		sleepms(1000);
		std::cout << blue_ << "\teStop() activated for 3 sec" << std::endl;
		controller.eStop();
		sleepms(3000);
		std::cout << "\teStop() released" << def_ << std::endl;
		controller.eStopRelease();
		sleepms(1000);
		controller.setMotorPower(0);
		sleepms(1000);
	}
	else
	{
		std::cout << red_ << "\tError: SmbController is not connected/ready. "<< def_ << std::endl;
	}
}

void testGetMode(smb_driver::SmbController& controller)
{
	std::cout << green_ << "***Testing getMode()***" << def_ << std::endl;
	if(controller.isConnected())
	{
		std::cout << blue_ << "\tMode returned " << controller.getMode() << def_ << std::endl;
	}
	else
	{
		std::cout << red_ << "\tError: SmbController is not connected/ready. "<< def_ << std::endl;
	}
}

void testVelocity(smb_driver::SmbController& controller)
{
	std::cout << green_ << "***Testing setVelocity()***" << def_ << std::endl;
	if(controller.isConnected())
	{
		std::cout << blue_ << "\tSetting velocity mode" << def_ << std::endl;
		controller.setMode(smb_driver::CLOSED_LOOP_SPEED);
		sleepms(1000);
		std::cout << blue_ << "\tSetting velocity to 50" << def_ << std::endl;
		controller.setVelocity(50);
		sleepms(1000);
		std::cout << blue_ << "\tSetting velocity to 0" << def_ << std::endl;
		controller.setVelocity(0);
		sleepms(1000);
	}
	else
	{
		std::cout << red_ << "\tError: SmbController is not connected/ready. "<< def_ << std::endl;
	}

}

int main(int argc, char *argv[])
{
#ifdef __XENO__
    // This is required for rt_printf()
    rt_print_auto_init(1);
    mlockall(MCL_CURRENT|MCL_FUTURE);
#endif // __XENO__

    ros::init(argc, argv, "SmbControllerTest");
    ros::NodeHandle nh;

	smb_driver::SmbController controller("/dev/ttySMB", nh);
	controller.startAcquisition();

	sleepms(1000); // required to make sure the roboteq

	testGetMode(controller);
	testMotorPower(controller);
	testVelocity(controller);
//	testEstop(controller);

	return 0;
}


