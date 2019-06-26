#ifndef __RoboteqDevice_H_
#define __RoboteqDevice_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sstream>
#include <unistd.h>
#include "smb_driver/ErrorCodes.h"

class RoboteqDevice
{
private:
	int handle;

protected:
	void InitPort();

	int Write(std::string str);
	int ReadAll(std::string &str);
//	int ReadAll(std::string &str, int timeoutUs);

	int IssueCommand(std::string commandType, std::string command, std::string args, int waitms, std::string &response, bool isplusminus = false);
	int IssueCommand(std::string commandType, std::string command, int waitms, std::string &response, bool isplusminus = false);

public:
	bool IsConnected();
	int Connect(std::string port);
	void Disconnect();

	int SetConfig(int configItem, int index, int value);
	int SetConfig(int configItem, int value);

	int SetCommand(int commandItem, int index, int value);
	int SetCommand(int commandItem, int value);
	int SetCommand(int commandItem);

	int GetConfig(int configItem, int index, int &result);
	int GetConfig(int configItem, int &result);

	int GetValue(int operatingItem, int index, int &result);
	int GetValue(int operatingItem, int &result);

	int TestFunc(int i);

	RoboteqDevice();
	~RoboteqDevice();
};

#endif
