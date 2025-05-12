#include <iostream>
#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sstream>

#ifdef _WIN32
#include <Windows.h>
#endif

#ifdef linux
#include <termios.h>
#include <unistd.h>
#endif

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;

#if _MSC_VER >= 1700 //Visual Studio 2012 or later
#define sprintf sprintf_s
#endif

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

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

int RoboteqDevice::IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus)
{
	int status;
	string read;
	response = "";

	if(args == "")
		status = Write(commandType + command + "\r");
	else
		status = Write(commandType + command + " " + args + "\r");

	if(status != RQ_SUCCESS)
		return status;

	sleepms(waitms);

	status = ReadAll(read);
	if(status != RQ_SUCCESS)
		return status;

	if(isplusminus)
	{
		if(read.length() < 2)
			return RQ_INVALID_RESPONSE;

		response = read.substr(read.length() - 2, 1);
		return RQ_SUCCESS;
	}

	string::size_type pos = read.rfind(command + "=");
	if(pos == string::npos)
		return RQ_INVALID_RESPONSE;

	pos += command.length() + 1;

	string::size_type carriage = read.find("\r", pos);
	if(carriage == string::npos)
		return RQ_INVALID_RESPONSE;

	response = read.substr(pos, carriage - pos);

	return RQ_SUCCESS;
}
int RoboteqDevice::IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus)
{
	return IssueCommand(commandType, command, "", waitms, response, isplusminus);
}

/*
	Set configuration parameter of the controller. Has overload that takes only 1 input. Relevant parameters are:

	_ALIM - set motor amp limit -> index = 1 for channel 1, 2 for channel 2; value = amps * 10
	example: SetConfig(_ALIM, 1, 305) sets channel 1 amp limit to 30.5 A

	_CLERD - set closed loop error detection -> index = 1 for channel 1, 2 for channel 2; value = detection mode
	NOTE: detection modes: 0 - detection disabled
						   1 - motor stops if (command - feedback) > 100 for more than 250ms
						   2 - motor stops if (command - feedback) > 250 for more than 500ms
						   3 - motor stops if (command - feedback) > 500 for more than 1000ms
	default is value = 2
	example: SetConfig(_CLERD, 2, 3) sets channel 2 motor to stop if (command - feedback) > 500 for more than 1000ms

	_EMOD - set encoder mode (unused or feedback) -> index = 1 for channel 1, 2 for channel 2; value = 18 for motor 1, 34 for motor 2
	default is unused
	example: SetConfig(_EMOD, 1, 18) sets encoder in channel 1 to act as feedback for motor 1

	_EPPR - set encoder pulse per revolution -> index = 1 for channel 1, 2 for channel 2; value = encoder PPR
	NOTE: there are 4 counts per pulse so the total number of counts is 4 x PPR
	default is 100 PPR
	example: SetConfig(_EPPR, 1, 400) sets encoder 1 PPR to 400 pulses per rotation

	_ICAP - set I limit as a percentage -> index = 1 for channel 1, 2 for channel 2; value = integral cap in %
	default is 100%
	example: SetConfig(_ICAP, 2, 30) sets channel 2 I limit to 30% <- percentage of what??

	_KD - set D gain -> index = 1 for channel 1, 2 for channel 2; value = D gain * 10
	default is 0
	example: SetConfig(_KD, 2, 10) sets channel 2 D gain to 1

	_KI - set I gain -> index = 1 for channel 1, 2 for channel 2; value = I gain * 10
	default is 0
	example: SetConfig(_KI, 2, 10) sets channel 2 I gain to 1

	_KP - set P gain -> index = 1 for channel 1, 2 for channel 2; value = P gain * 10
	default is 0
	example: SetConfig(_KP, 2, 10) sets channel 2 P gain to 1

	_MDIR - set motor direction -> index = 1 for channel 1, 2 for channel 2; value = 0 for not inverted, 1 for inverted
	default is not inverted
	example: SetConfig(_MDIR, 2, 1) sets channel 2 to inverted motor direction

	_MAC/_MDEC - set motor acceleration/deceleration -> index = 1 for channel 1, 2 for channel 2; value = acceleration in 0.1rpm per second
	NOTE: in closed loop torque control, value = mA per second
	default is 1000rpm/s
	example: SetConfig(_MAC, 2, 15000) sets channel 2 acceleration to 1500rpm/s

	_MMOD - set motor operating mode -> index = 1 for channel 1, 2 for channel 2; value = operating mode
	NOTE: operating modes: 0 - open loop
						   1 - closed loop speed
						   2 - close loop position relative
						   3 - closed loop count position
						   4 - closed loop position tracking
						   5 - closed loop torque
						   6 - closed loop speed and position
	default is open loop
	example: SetConfig(_MMOD, 1, 5) sets channel 1 to closed loop torque control

	_MVEL - set motor speed (RPM) for position mode -> index = 1 for channel 1, 2 for channel 2; value = rpm
	default is 1000rpm
	example: SetConfig(_MVEL, 1, 3500) sets channel 1 RPM in position control to 3500rpm

	_MXRPM - set max speed (RPM) -> index = 1 for channel 1, 2 for channel 2; value = rpm
	default is 1000rpm
	example: SetConfig(_MXRPM, 1, 2900) sets channel 1 max RPM to 2900rpm
*/
int RoboteqDevice::SetConfig(int configItem, int index, int value)
{
	string response;
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

	int status = IssueCommand("^", command, args, 10, response, true);
	if(status != RQ_SUCCESS)
		return status;
	if(response != "+")
		return RQ_SET_CONFIG_FAILED;

	return RQ_SUCCESS;
}

/*
	Set configuration parameter of the controller. Overloaded version that only takes 1 input. Relevant parameters are:

	_ECHOF - set echo for serial communication -> value = 0 for enable, 1 for disable
	default is enabled
	example: SetConfig(_ECHOF, 1) disables serial echo

	_RWD - set watchdog timeout for serial communication -> index = 1 for channel 1, 2 for channel 2; value = timeout value in ms
	default is 1 second
	example: SetConfig(_RWD, 3000) sets watchdog to 3 seconds
*/
int RoboteqDevice::SetConfig(int configItem, int value)
{
	return SetConfig(configItem, MISSING_VALUE, value);
}

/*
	Send runtime commands to the motor controller. Has overloads that take 2 inputs or 1 input. Relevant commands are:

	_AC/_ACCEL - set motor acceleration during runtime -> index = 1 for channel 1, 2 for channel 2; value = Acceleration in 0.1*RPM/s
	NOTE:	- Identical to the MAC but is provided so that it can be changed rapidly during motor operation.
			- Not applicable if either of the MAC or MDEC is set to 0.
	example: SetCommand(_AC, 1，2000) Increase Motor 1 speed by 200 RPM every second

	_C/_SENCNTR - set encoder count -> index = 1 for channel 1, 2 for channel 2; value = Counter value (Min: -2147M Max: +2147M)
	example: SetCommand(_C, 2，-1000) Loads -1000 in encoder counter 2
			 SetCommand(_C, 1，0) Clears encoder counter 1

	_DC/_DECEL - set motor decceleration during runtime -> index = 1 for channel 1, 2 for channel 2; value = Deceleration in 0.1*RPM/s
	NOTE:	- Identical to the DMAC but is provided so that it can be changed rapidly during motor operation.
			- Not applicable if either of the MAC or MDEC is set to 0.
	example: SetCommand(_DC, 1，2000) Reduce Motor 1 speed by 200 RPM every second

	_EX/_ESTOP - emergency stop 
	example: SetCommand(_EX, 1)

	_MG/_MGO - Emergency Stop Release
	example: SetCommand(_MG, 1)

	_G/_GO - go to speed or to relative position -> index = 1 for channel 1, 2 for channel 2; value ranges from -1000 to +1000
	NOTE: The G command has no effect in the Position Count mode.
	example: - SetCommand(_G, 1，500) In Open Loop Speed mode, applies 50% power to motor channel 1
			 - SetCommand(_G, 1，500) In Closed Loop Speed mode, assuming that 3000 is contained in Max RPM parameter (MXRPM), motor will go to 1500 RPM
			 - SetCommand(_G, 1，500) In Closed Loop Relative or Closed Loop Tracking modes, the motor will move to 75% position of the total -1000 to +1000 motion range
			 - SetCommand(_G, 1，500) In Torque mode, assuming that Amps Limit is 60A, motor power will rise until 30A are measured.

	_MS/_MSTOP - stop the motor for the specified motor channel -> index = Motor channel
	example: SetCommand(_MS, cc)

	_P/_MOTPOS - Go to Absolute Desired Position -> index = 1 for channel 1, 2 for channel 2; value = Absolute count destination (Min: -2147M Max: +2147M)
	NOTE: This command is used in the Position Count mode to make the motor move to a specified feedback sensor count value.
	example: SetCommand(_P, 1, 10000) Make motor go to absolute count value 10000.

	_PR/_MPOSREL - Go to Relative Desired Position -> index = 1 for channel 1, 2 for channel 2; value = Relative count position (Min: -2147M Max: +2147M)
	Note: counter will rollover at counter values +/-2’147’483’648.
	example: SetCommand(_PR, 2, 10000) while previous command was absolute goto position !P 2 5000, motor will go to +15000

	_S/_MOTVEL - Set Motor Speed  ->  index = 1 for channel 1, 2 for channel 2; value = Speed in RPM (Min:-65535 Max: 65535)
	NOTE: 	Closed-Loop Speed mode - cause the motor to spin at the desired RPM speed. 
			Closed-Loop Position modes - determines the speed at which the motor will move from one position to the next. It will not actually start the motion.
	example: SetCommand(_S, 1, 2500) Set motor 1 position velocity to 2500 RPM


*/
int RoboteqDevice::SetCommand(int commandItem, int index, int value)
{
	string response;
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

	int status = IssueCommand("!", command, args, 10, response, true);
	if(status != RQ_SUCCESS)
		return status;
	if(response != "+")
		return RQ_SET_COMMAND_FAILED;

	return RQ_SUCCESS;
}

/*
	Send runtime commands to the motor controller. Overloaded version that only takes 2 inputs. Relevant commands are:
*/
int RoboteqDevice::SetCommand(int commandItem, int value)
{
	return SetCommand(commandItem, MISSING_VALUE, value);
}

/*
	Send runtime commands to the motor controller. Overloaded version that only takes 1 input. Relevant commands are:
*/
int RoboteqDevice::SetCommand(int commandItem)
{
	return SetCommand(commandItem, MISSING_VALUE, MISSING_VALUE);
}

/*
	Get controller configuration parameter. Has overload that takes 1 input. See SetConfig for relevant parameters.
*/
int RoboteqDevice::GetConfig(int configItem, int index, int &result)
{
	string response;
	char command[10];
	char args[50];

	if(configItem < 0 || configItem > 255)
		return RQ_INVALID_CONFIG_ITEM;

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	sprintf(command, "$%02X", configItem);
	sprintf(args, "%i", index);

	int status = IssueCommand("~", command, args, 10, response);
	if(status != RQ_SUCCESS)
		return status;

	istringstream iss(response);
	iss>>result;

	if(iss.fail())
		return RQ_GET_CONFIG_FAILED;

	return RQ_SUCCESS;
}

/*
	Get controller configuration parameter. Overloaded version that takes 1 input. See SetConfig for relevant parameters.
*/
int RoboteqDevice::GetConfig(int configItem, int &result)
{
	return GetConfig(configItem, 0, result);
}

/*
	Get runtime operating values from the motor controller. Has overload that takes 2 inputs. Relevant commands are:

	_A/_MOTAMPS - Read Motor Amps
				- Argument: [Motor channel]
				- Reply: Amps *10 for each channel
	example :   result = GetValue(_A, 1)

	_C/_ABCNTR  - Read Encoder Counter Absolute
				- Argument: [Motor channel]
				- Reply: Absolute counter value (Min: -2147M Max: 2147M)
	example :   result = GetValue(_C, 1)

	_E/_LPERR - Read Closed Loop Error: In closed-loop modes, returns the difference between the desired speed or position and the measured feedback.
			  - Argument: [Motor channel]
			  - Reply: Error value (Min: -2147M Max: 2147M)
	example :   result = GetValue(_E, 1)

	_P/_MOTPWR - Read Motor Power Output Applied
				- Argument: [Motor channel]
				- Reply: 0 to +/-1000 power level, A value of 1000 equals 100% PWM
	example :   result = GetValue(_P, 1)

	_S/_ABSPEED - Read Encoder Motor Speed in RPM: Reports the actual speed measured by the encoders as the actual RPM value.
				- Argument: [Motor channel]
				- Reply: Speed in RPM
	example :   result = GetValue(_S, 1)
	
*/
int RoboteqDevice::GetValue(int operatingItem, int index, int &result)
{
	string response;
	char command[10];
	char args[50];

	if(operatingItem < 0 || operatingItem > 255)
		return RQ_INVALID_OPER_ITEM;

	if(index < 0)
		return RQ_INDEX_OUT_RANGE;

	sprintf(command, "$%02X", operatingItem);
	sprintf(args, "%i", index);

	int status = IssueCommand("?", command, args, 10, response);
	if(status != RQ_SUCCESS)
		return status;

	istringstream iss(response);
	iss>>result;

	if(iss.fail())
		return RQ_GET_VALUE_FAILED;

	return RQ_SUCCESS;
}

/*
	Get runtime operating values from the motor controller. Overloaded version that takes 2 inputs. Relevant commands are:
*/
int RoboteqDevice::GetValue(int operatingItem, int &result)
{
	return GetValue(operatingItem, 0, result);
}

string ReplaceString(string source, string find, string replacement)
{
	string::size_type pos = 0;
    while((pos = source.find(find, pos)) != string::npos)
	{
        source.replace(pos, find.size(), replacement);
        pos++;
    }

	return source;
}

#ifdef _WIN32
int RoboteqDevice::Connect(string port)
{
	if (IsConnected())
	{
		cout << "Device is connected, attempting to disconnect." << endl;
		Disconnect();
	}

	//Open port.
	cout << "Opening port: '" << port << "'...";
	HANDLE h = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	handle = (int)h;
	if (h == INVALID_HANDLE_VALUE)
	{
		handle = RQ_INVALID_HANDLE;
		cout << "failed." << endl;
		return RQ_ERR_OPEN_PORT;
	}

	cout << "succeeded." << endl;

	cout << "Initializing port...";
	InitPort();
	cout << "...done." << endl;

	int status;
	string response;
	cout << "Detecting device version...";
	status = IssueCommand("?", "$1E", 10, response);
	//status = IssueCommand("?", "FID", 10, response);
	if (status != RQ_SUCCESS)
	{
		cout << "failed (issue ?$1E response: " << status << ")." << endl;
		Disconnect();
		return RQ_UNRECOGNIZED_DEVICE;
	}

	if (response.length() < 12)
	{
		cout << "failed (unrecognized version)." << endl;
		Disconnect();
		return RQ_UNRECOGNIZED_VERSION;
	}

	cout << response.substr(8, 4) << "." << endl;
	return RQ_SUCCESS;
}
void RoboteqDevice::Disconnect()
{
	if (IsConnected())
		CloseHandle((HANDLE)handle);

	handle = RQ_INVALID_HANDLE;
}
void RoboteqDevice::InitPort()
{
	if (!IsConnected())
		return;

	DCB          comSettings;
	COMMTIMEOUTS CommTimeouts;

	// Set timeouts in milliseconds
	CommTimeouts.ReadIntervalTimeout = 0;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadTotalTimeoutConstant = 100;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	CommTimeouts.WriteTotalTimeoutConstant = 100;
	SetCommTimeouts((HANDLE)handle, &CommTimeouts);

	// Set Port parameters.
	// Make a call to GetCommState() first in order to fill
	// the comSettings structure with all the necessary values.
	// Then change the ones you want and call SetCommState().
	GetCommState((HANDLE)handle, &comSettings);
	comSettings.BaudRate = 115200;
	comSettings.StopBits = ONESTOPBIT;
	comSettings.ByteSize = 8;
	comSettings.Parity = NOPARITY;
	comSettings.fParity = FALSE;
	SetCommState((HANDLE)handle, &comSettings);
}
int RoboteqDevice::Write(string str)
{
	if (!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	//cout<<"Writing: "<<ReplaceString(str, "\r", "\r\n");
	DWORD written = 0;
	int bStatus = WriteFile((HANDLE)handle, str.c_str(), str.length(), &written, NULL);
	if (bStatus == 0)
		return RQ_ERR_TRANSMIT_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::ReadAll(string &str)
{
	DWORD countRcv;
	if (!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	char buf[BUFFER_SIZE + 1] = "";

	str = "";
	int i = 0;

	while (ReadFile((HANDLE)handle, buf, BUFFER_SIZE, &countRcv, NULL) != 0)
	{
		str.append(buf, countRcv);

		//No further data.
		if (countRcv < BUFFER_SIZE)
			break;
	}

	//if(countRcv > 0)
	//{
	//	str.append(buf, countRcv);
	//}

	return RQ_SUCCESS;
}
void sleepms(int milliseconds)
{
	Sleep(milliseconds);
}
#endif

#ifdef linux
int RoboteqDevice::Connect(string port)
{
	if (IsConnected())
	{
		cout << "Device is connected, attempting to disconnect." << endl;
		Disconnect();
	}

	//Open port.
	cout << "Opening port: '" << port << "'...";
	handle = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (handle == RQ_INVALID_HANDLE)
	{
		cout << "failed." << endl;
		return RQ_ERR_OPEN_PORT;
	}

	cout << "succeeded." << endl;
	fcntl(handle, F_SETFL, O_APPEND | O_NONBLOCK & ~FNDELAY);

	cout << "Initializing port...";
	InitPort();
	cout << "...done." << endl;

	int status, i;
	string response;
	cout << "Detecting device version...";

	for (i = 0; i < 5; i++)
	{
		status = IssueCommand("?", "$1E", 50, response);
		if (status == RQ_SUCCESS)
			break;
	}

	if (status != RQ_SUCCESS)
	{
		cout << "failed (issue ?$1E response: " << status << ")." << endl;
		Disconnect();
		return RQ_UNRECOGNIZED_DEVICE;
	}

	if (response.length() < 12)
	{
		cout << "failed (unrecognized version)." << endl;
		Disconnect();
		return RQ_UNRECOGNIZED_VERSION;
	}

	cout << response.substr(8, 4) << "." << endl;
	return RQ_SUCCESS;
}
void RoboteqDevice::Disconnect()
{
	if (IsConnected())
		close(handle);

	handle = RQ_INVALID_HANDLE;
}
void RoboteqDevice::InitPort()
{
	if (!IsConnected())
		return;

	//Get the existing Comm Port Attributes in cwrget
	int BAUDRATE = B115200;
	struct termios newtio;
	tcgetattr(handle, &newtio);

	//Set the Tx and Rx Baud Rate to 115200
	cfsetospeed(&newtio, (speed_t)BAUDRATE);
	cfsetispeed(&newtio, (speed_t)BAUDRATE);

	//Enable the Receiver and  Set local Mode
	newtio.c_iflag = IGNBRK;		/* Ignore Break Condition & no processing under input options*/
	newtio.c_lflag = 0;			/* Select the RAW Input Mode through Local options*/
	newtio.c_oflag = 0;			/* Select the RAW Output Mode through Local options*/
	newtio.c_cflag |= (CLOCAL | CREAD);	/* Select the Local Mode & Enable Receiver through Control options*/

										//Make RAW Mode more explicit by turning Canonical Mode off, Echo off, Echo Erase off and Signals off*/
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	//Disable Software Flow Control
	newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

	//Set Data format to 8N1
	newtio.c_cflag &= ~CSIZE;		/* Mask the Character Size Bits through Control options*/
	newtio.c_cflag |= CS8;			/* Select Character Size to 8-Bits through Control options*/
	newtio.c_cflag &= ~PARENB;		/* Select Parity Disable through Control options*/
	newtio.c_cflag &= ~PARODD;		/* Select the Even Parity (Disabled) through Control options*/
	newtio.c_cflag &= ~CSTOPB;		/*Set number of Stop Bits to 1*/

									//Timout Parameters. Set to 0 characters (VMIN) and 10 second (VTIME) timeout. This was done to prevent the read call from blocking indefinitely.*/
	newtio.c_cc[VMIN] = 0;
	newtio.c_cc[VTIME] = 100;

	/* Flush the Input buffer and set the attribute NOW without waiting for Data to Complete*/
	tcflush(handle, TCIFLUSH);
	tcsetattr(handle, TCSANOW, &newtio);
}
int RoboteqDevice::Write(string str)
{
	if (!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	//cout<<"Writing: "<<ReplaceString(str, "\r", "\r\n");
	int countSent = write(handle, str.c_str(), str.length());

	//Verify weather the Transmitting Data on UART was Successful or Not
	if (countSent < 0)
		return RQ_ERR_TRANSMIT_FAILED;

	return RQ_SUCCESS;
}
int RoboteqDevice::ReadAll(string &str)
{
	int countRcv;
	if (!IsConnected())
		return RQ_ERR_NOT_CONNECTED;

	char buf[BUFFER_SIZE + 1] = "";

	str = "";
	int i = 0;
	while ((countRcv = read(handle, buf, BUFFER_SIZE)) > 0)
	{
		str.append(buf, countRcv);

		//No further data.
		if (countRcv < BUFFER_SIZE)
			break;
	}

	if (countRcv < 0)
	{
		if (errno == EAGAIN)
			return RQ_ERR_SERIAL_IO;
		else
			return RQ_ERR_SERIAL_RECEIVE;
	}

	return RQ_SUCCESS;
}
void sleepms(int milliseconds)
{
	usleep(milliseconds / 1000);
}
#endif
