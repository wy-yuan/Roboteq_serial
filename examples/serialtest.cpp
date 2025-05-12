#include <iostream>
#include <stdio.h>
#include <string.h>

#include "ErrorCodes.h"
#include "Constants.h"
#include "RoboteqDriver.h"
#include "serial/serial.h"

using namespace std;

class RoboteqDriver
{
public:
    
	RoboteqDriver()
	{
	}

	~RoboteqDriver()
	{
		if (ser.isOpen())
		{
			ser.close();
		}
	}
	serial::Serial ser;
	void connect(string port, int32_t baud)
	{
		try
		{
			ser.setPort(port);
			ser.setBaudrate(baud); //get baud as param
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			ser.open();
		}
		catch (serial::IOException &e)
		{
			cout << "Unable to open port " << endl;
			
		}
		if (ser.isOpen())
		{
			cout << "Serial Port initialized" << endl;
		}
		else
		{
			cout << "Serial Port is not open" << endl;
		}
	}

	// void cmd_vel_callback()
	// {
	// 	std::stringstream cmd_sub;
	// 	cmd_sub << "!G 1 10";

	// 	ser.write(cmd_sub.str());
	// 	ser.flush();
	// }

};

int serialTest(string port){
    RoboteqDriver driver;
    driver.connect(port, 115200);
    return 0;
}

int main(int argc, char *argv[])
{

#ifdef _WIN32
	string port = "\\\\.\\com4";
#endif

#ifdef linux
	string port = "/dev/ttyS4";
#endif

	return serialTest(port);
}