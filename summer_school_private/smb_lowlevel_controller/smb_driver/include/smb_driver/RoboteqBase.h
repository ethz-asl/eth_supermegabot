/*
 * RoboteqBase.h
 *
 *  Created on: May 24, 2017
 *      Author: sasutosh
 */

#ifndef INCLUDE_ROBOTEQBASE_H_
#define INCLUDE_ROBOTEQBASE_H_

#include<stddef.h>
#include<iostream>

class RoboteqBase {
public:

    RoboteqBase() {};
    virtual ~RoboteqBase() {};

    virtual bool IsConnected() = 0;
	virtual int Connect(std::string port) =0;
	virtual void Disconnect() = 0;

	virtual int SetConfig(int configItem, int index, int value) = 0;  
	virtual int SetConfig(int configItem, int value) = 0;

	virtual int SetCommand(int commandItem, int index, int value) = 0;
	virtual int SetCommand(int commandItem, int value) = 0;
	virtual int SetCommand(int commandItem) = 0;

	virtual int GetConfig(int configItem, int index, int &result) = 0;
	virtual int GetConfig(int configItem, int &result) = 0;

	virtual int GetValue(int operatingItem, int index, int &result) = 0;
	virtual int GetValue(int operatingItem, int &result) = 0;
	virtual int TestFunc(int i) = 0;

};


#endif /* INCLUDE_ROBOTEQBASE_H_ */
