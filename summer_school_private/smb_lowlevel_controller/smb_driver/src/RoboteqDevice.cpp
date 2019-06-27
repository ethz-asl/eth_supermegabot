

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

#include "smb_driver/RoboteqDevice.h"

RoboteqDevice::RoboteqDevice()
{
	handle = RQ_INVALID_HANDLE;
}
RoboteqDevice::~RoboteqDevice()
{
	Disconnect();
}

bool RoboteqDevice::IsConnected()
{
	return handle != RQ_INVALID_HANDLE;
}
int RoboteqDevice::Connect(std::string port)
{
	if(IsConnected())
	{
		printf("Device is connected, attempting to disconnect.\n");
		Disconnect();
	}

	//Open port.
	printf("Opening port: '%s'... ", port.c_str());
	handle = open(port.c_str(), O_RDWR |O_NOCTTY | O_NDELAY);
	if(handle == RQ_INVALID_HANDLE)
	{
		printf("failed.\n");
		return RQ_ERR_OPEN_PORT;
	}

	printf("succeeded.\n");
	fcntl (handle, F_SETFL, O_APPEND | O_NONBLOCK);

	printf("Initializing port...");
	InitPort();
	printf("...done.\n");

	int status = -1;
	std::string response;
//	printf("Detecting device version...");
	status = IssueCommand("?", "$1E", 1, response);
//	printf("failed (issue ?$1E response: %i ).\n", status);

	if(status != RQ_SUCCESS)
	{
		printf("failed (issue ?$1E response: %i ).\n", status);
//		Disconnect();
//		return RQ_UNRECOGNIZED_DEVICE;
	}
//
//	printf("%s.\n", response.substr(8, 4).c_str());
	return RQ_SUCCESS;
}

void RoboteqDevice::Disconnect()
{
	if(IsConnected())
		close(handle);

	handle = RQ_INVALID_HANDLE;
}

void RoboteqDevice::InitPort()
{
	if(!IsConnected())
		return;

	//Get the existing Comm Port Attributes in cwrget
	int BAUDRATE = B115200;
	struct termios newtio;
	tcgetattr (handle, &newtio);

	//Set the Tx and Rx Baud Rate
	cfsetospeed (&newtio, (speed_t)BAUDRATE);
	cfsetispeed (&newtio, (speed_t)BAUDRATE);
//--------------------------
	//Enable the Receiver and  Set local Mode
	newtio.c_iflag = IGNBRK;		/* Ignore Break Condition & no processing under input options*/
	newtio.c_lflag = 0;			/* Select the RAW Input Mode through Local options*/
	newtio.c_oflag = 0;			/* Select the RAW Output Mode through Local options*/
	newtio.c_cflag |= (CLOCAL | CREAD);	/* Select the Local Mode & Enable Receiver through Control options*/

	//Set Data format to 7E1
	newtio.c_cflag &= ~CSIZE;		/* Mask the Character Size Bits through Control options*/
//	newtio.c_cflag &= ~(CSIZE | PARENB);	/* Parity bit ? */
	newtio.c_cflag |= CS8;			/* Select Character Size*/

	// experimental.. maybe works!!!!
	newtio.c_cflag &= ~CSTOPB;            // 1 stop bit
	newtio.c_cflag &= ~CRTSCTS;           // Disable hardware flow control

	//Set the New Comm Port Attributes through cwrset
	tcsetattr (handle, TCSANOW, &newtio);	/* Set the attribute NOW without waiting for Data to Complete*/
}

int RoboteqDevice::Write(std::string str)
{
	if(!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	//std::cout << "command: " << str << std::endl;

	//cout<<"Writing: "<<ReplaceString(str, "\r", "\r\n");
	int countSent = write(handle, str.c_str(), str.length());

	//Verify weather the Transmitting Data on UART was Successful or Not
	if(countSent < 0)
		return RQ_ERR_TRANSMIT_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::ReadAll(std::string &str)
{
	int countRcv;
	if(!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	char buf[BUFFER_SIZE + 1] = "";

	str = "";
	while((countRcv = read(handle, buf, BUFFER_SIZE)) > 0)
	{
		str.append(buf, countRcv);

		//No further data.
		if(countRcv < BUFFER_SIZE)
			break;
	}

	if(countRcv < 0)
	{
		if(errno == EAGAIN)
			return RQ_ERR_SERIAL_IO;
		else
			return RQ_ERR_SERIAL_RECEIVE;
	}

	return RQ_SUCCESS;
}

//int RoboteqDevice::ReadAll(std::string &str, int timeoutUs)
//{
//	int countRcv;
//	if(!IsConnected())
//		return RQ_ERR_NOT_CONNECTED;
//
//	char buf[BUFFER_SIZE + 1] = "";
//
//	str = "";
//	int time = 0;
//	while(time < timeoutUs) {
//		countRcv = read(handle, buf, BUFFER_SIZE);
//std::cout << "read " << countRcv << " bytes. ";
//		str.append(buf, countRcv);
//
//		//If there is a carriage return, the message is complete
//		if(read.find("\r") != std::string::npos)
//			break;
//		else {
//			time += 100;
//			usleep(100);
//		}
//	}
//
//	if(countRcv < 0)
//	{
//		if(errno == EAGAIN)
//			return RQ_ERR_SERIAL_IO;
//		else
//			return RQ_ERR_SERIAL_RECEIVE;
//	}
//
//	return RQ_SUCCESS;
//}

int RoboteqDevice::IssueCommand(std::string commandType, std::string command, std::string args, int waitms, std::string &response, bool isplusminus)
{
	int status;
	std::string str;
	response = "";

	if(args == "")
		status = Write(commandType + command + "\r");
	else
		status = Write(commandType + command + " " + args + "\r");

	if(status != RQ_SUCCESS)
		return status;

	//Read the response from the controller
	int countRcv;
	if(!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	char buf[BUFFER_SIZE + 1] = "";

	str = "";
	int time = 0;
	int progress = 0;
	std::string::size_type returnStartPos;

	//todo waitms should be used here, not 5000, right???
	while(time < 500000) {
		countRcv = read(handle, buf, BUFFER_SIZE);

		if (countRcv <= 0) {
			if (countRcv < 0) {
				if(errno != EAGAIN) //This means read was interrupted by a signal before any bites were read
					return RQ_ERR_SERIAL_RECEIVE;
			}
			time += 10000;
			usleep(10000);
			continue;
		}


		str.append(buf, countRcv);
//std::cout << "str " << str << std::endl;
		//First look for the echo of the command
		if (progress == 0) {
			returnStartPos = str.rfind(command);
			if(returnStartPos != std::string::npos) {
				returnStartPos += command.length() + 1; //Move to the character after the '=' for messages with a value response
				progress = 1;
//				std::cout << "Command echo received" << std::endl;
			}
			else
				continue;
		}

		//Next look for the carriage return, signaling the end of the message
		if (progress == 1) {
			std::string::size_type carriage;

			if (isplusminus) {
				carriage = str.rfind("\r"); //For the motor commands, there are multiple returns in each reply
				if(carriage != std::string::npos) {
					response = str.substr(carriage-1, 1);
//std::cout << "response: " << response << " read in " << time << " microsec" << std::endl;
					return RQ_SUCCESS;
				}
			}
			else {
				carriage = str.find("\r", returnStartPos);
				if(carriage != std::string::npos) {
					response = str.substr(returnStartPos, carriage - returnStartPos);
//std::cout << "response: " << response << " read in " << time << " microsec" << std::endl;
					return RQ_SUCCESS;
				}
			}
		}

		//If not done yet, sleep then try reading again
		time += 100;
		usleep(100);
	}

	//If we get here, the read timed out
//	std::cout << "Read timed out!!! str:" << str << std::endl;
	return RQ_ERR_SERIAL_RECEIVE;
}

int RoboteqDevice::IssueCommand(std::string commandType, std::string command, int waitms, std::string &response, bool isplusminus)
{
	return IssueCommand(commandType, command, "", waitms, response, isplusminus);
}

int RoboteqDevice::SetConfig(int configItem, int index, int value)
{
	std::string response;
	char command[10];
	char args[50];

	if(configItem < 0 || configItem > 255)
		return RQ_INVALID_CONFIG_ITEM;

	sprintf(command, "$%02X", configItem);
	sprintf(args, "%i %i", index, value);
	if(index == MISSING_VALUE)
	{
		sprintf(args, "%i", value);
		index = 0;
	}

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	int status = IssueCommand("^", command, args, 1, response, true);
  //printf("Roboteq IssueCommand gave response: %s \n", response.c_str());
  if(status != RQ_SUCCESS)
		return status;
	if(response != "+"){
    return RQ_SET_CONFIG_FAILED;
  }

	return RQ_SUCCESS;
}
int RoboteqDevice::SetConfig(int configItem, int value)
{
	return SetConfig(configItem, MISSING_VALUE, value);
}

int RoboteqDevice::SetCommand(int commandItem, int index, int value)
{
	std::string response;
	char command[10];
	char args[50];

	if(commandItem < 0 || commandItem > 255)
		return RQ_INVALID_COMMAND_ITEM;

	sprintf(command, "$%02X", commandItem);
	sprintf(args, "%i %i", index, value);
	if(index == MISSING_VALUE)
	{
		if(value != MISSING_VALUE)
			sprintf(args, "%i", value);
		index = 0;
	}

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	int status = IssueCommand("!", command, args, 2, response, true);
  //printf("Roboteq IssueCommand gave response: %s \n", response.c_str());
  if(status != RQ_SUCCESS)
		return status;
	if(response != "+") {
		return RQ_SET_COMMAND_FAILED;
	}

	return RQ_SUCCESS;
}
int RoboteqDevice::SetCommand(int commandItem, int value)
{
	return SetCommand(commandItem, MISSING_VALUE, value);
}
int RoboteqDevice::SetCommand(int commandItem)
{
	return SetCommand(commandItem, MISSING_VALUE, MISSING_VALUE);
}

int RoboteqDevice::GetConfig(int configItem, int index, int &result)
{
	std::string response;
	char command[10];
	char args[50];

	if(configItem < 0 || configItem > 255)
		return RQ_INVALID_CONFIG_ITEM;

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	sprintf(command, "$%02X", configItem);
	sprintf(args, "%i", index);

	int status = IssueCommand("~", command, args, 1, response);

  //printf("Roboteq GetConfig gave response: %s \n", response.c_str());

  if(status != RQ_SUCCESS)
		return status;

  std::istringstream iss(response);
	iss>>result;

	if(iss.fail())
		return RQ_GET_CONFIG_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::GetConfig(int configItem, int &result)
{
	return GetConfig(configItem, 0, result);
}

int RoboteqDevice::GetValue(int operatingItem, int index, int &result)
{
	std::string response;
	char command[10];
	char args[50];

	if(operatingItem < 0 || operatingItem > 255)
		return RQ_INVALID_OPER_ITEM;

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	sprintf(command, "$%02X", operatingItem);
	sprintf(args, "%i", index);

	int status = IssueCommand("?", command, args, 2, response);

  //printf("Roboteq GetValue gave response: %s \n", response.c_str());

  if(status != RQ_SUCCESS)
		return status;

	std::istringstream iss(response);
	iss>>result;

	if(iss.fail())
		return RQ_GET_VALUE_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::GetValue(int operatingItem, int &result)
{
	return GetValue(operatingItem, 0, result);
}


int RoboteqDevice::TestFunc(int i)
{
	return 100;
}

