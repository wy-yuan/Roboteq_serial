#include <iostream>
#include <stdio.h>
#include <string.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"
#include "serial/serial.h"

using namespace std;

int basicSample(string port) 
{
	cout << endl << "Motor Controller Basic Sample:" << endl;
	RoboteqDevice device;
	int status = device.Connect(port);

	if (status != RQ_SUCCESS)
	{
		cout << "Error connecting to device: " << status << "." << endl;
		return 1;
	}

    // set to closed loop count position
    device.SetConfig(_MMOD, 1, 3);
    // set encoder mode as feedback
    device.SetConfig(_EMOD, 1, 18); 
    // set encoder pulse per revolution
    device.SetConfig(_EPPR, 1, 1855); // 1855=7420/4
    // set PID = 1,1,1
    device.SetConfig(_KP, 1, 10);
    device.SetConfig(_KD, 1, 10);
    device.SetConfig(_KI, 1, 10);
    // set motor direction
    device.SetConfig(_MDIR, 1, 0);

    // read config and print
    int mode, enc_fd, PPR, P, I, D;
    device.GetConfig(_MMOD, 1, mode);
    cout << "Motor operating mode: " << mode << "." << endl;
    device.GetConfig(_EMOD, 1, enc_fd);
    if(enc_fd == 18){
        cout << "Encoder 1 is used as feedback for motor 1." << endl;
    }else if(enc_fd == 34){
        cout << "Encoder 1 is used as feedback for motor 2." << endl;
    }else{
        cout << "Encoder 1 is not used as feedback." << endl;
    }
    device.GetConfig(_EPPR, 1, PPR);
    cout << "Encoder pulse per revolution: " << PPR << "." << endl;
    device.GetConfig(_KP, 1, P);
    device.GetConfig(_KD, 1, D);
    device.GetConfig(_KI, 1, I);
    cout << "PID values: " << P/10 << "," << I/10 << "," << D/10 << endl;

    /*
    // Clears encoder counter
    device.SetCommand(_C, 1, 0);
    // Go to Absolute Desired Position in position count mode
    device.SetCommand(_P, 1, 1000);
    sleepms(1000);  // wait the motor moving to the desired position
    // read encoder counter and print
    int result;
    device.GetValue(_C, 1, result);
    cout << "encoder counter: " << result << "." << endl;
    // stop the motor
    device.SetCommand(_MS, 1);
    */

	device.Disconnect();
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

	return basicSample(port);
}