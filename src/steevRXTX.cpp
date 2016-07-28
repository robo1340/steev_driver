#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include "serial/serial.h"
#include <vector>
#include <map>

#include "steev_driver/Sensor.h"
#include "steev_driver/MechanicalOutputDevice.h"
#include "steev_driver/steevDriverConstants.h"

#include "steev_driver/ValueRequest.h"
#include "steev_driver/ParameterValueRequest.h"
#include "steev_driver/SetValueRequest.h"
#include "steev_driver/SetParameterValueRequest.h"
#include "steev_driver/SteevValue.h"
#include "steev_driver/SteevParameterValue.h"

using namespace steev_constants;
using std::string;
using std::stringstream;
using std::exception;
using std::cout;
using std::cin;
using std::cerr;
using std::endl;
using std::vector;
using std::map;

#define MAX_LINE_LENGTH 50
#define DEBUG 0
#define PARAM_CHAR '.'
#define ASSIGN_CHAR '='
#define CARRIAGE_NEWLINE "\r\n"
#define CARRIAGE_CHAR '\r'
#define NEWLINE_CHAR '\n'

serial::Serial* arduino_serial_ptr;

//intermediate variables used to hold field values of received messages
string id = "";  //id of the sensor or output device being selected
string pid = ""; //id of the parameter of teh sensor or output device being selected
int val = 0; //value being set to a sensor or output device

//the string to be sent over serial
stringstream serialRequest;

//counter variable for for-loops
int i=0;

void valueRequestCallback(const steev_driver::ValueRequest::ConstPtr& msg)
{
	if(DEBUG) { cout << "value request received" << endl;}
	id = msg->id;
	serialRequest << id  << endl;
	arduino_serial_ptr->write(serialRequest.str());
	serialRequest.str(string());//clear the string buffer before next use
}
void parameterValueRequestCallback(const steev_driver::ParameterValueRequest::ConstPtr& msg)
{
        if(DEBUG) { cout << "parameter value request received" << endl;}
	id = msg->id;
	pid = msg->paramID;
        serialRequest << id << PARAM_CHAR << pid << endl;
        arduino_serial_ptr->write(serialRequest.str());
        serialRequest.str(string());//clear the string buffer before next use
}
void setValueRequestCallback(const steev_driver::SetValueRequest::ConstPtr& msg)
{
        if(DEBUG) { cout << "set value request received" << endl;}
	id = msg->id;
	val = msg->val;
        serialRequest << id  << ASSIGN_CHAR << val << endl;
        arduino_serial_ptr->write(serialRequest.str());
        serialRequest.str(string());//clear the string buffer before next use
}
void setParameterValueRequestCallback(const steev_driver::SetParameterValueRequest::ConstPtr& msg)
{
        if(DEBUG) { cout << "set parameter value request received" << endl;}
	id = msg->id;
	pid = msg->paramID;
	val = msg->val;
        serialRequest << id << PARAM_CHAR << pid << ASSIGN_CHAR << val << endl;
        arduino_serial_ptr->write(serialRequest.str());
        serialRequest.str(string());//clear the string buffer before next use
}

//////////////////////////////////////////////////////////////////
//MAIN////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

  ros::init(argc, argv, "receiver");
  ros::NodeHandle n;

  ros::Publisher valuePub
    = n.advertise<steev_driver::SteevValue>("steev_value",100);
  ros::Publisher parameterValuePub
    = n.advertise<steev_driver::SteevParameterValue>("steev_parameter_value",100);

  ros::Subscriber valueRequestSub
    = n.subscribe("value_requests",100,valueRequestCallback);
  ros::Subscriber ParameterValueRequestSub
    = n.subscribe("parameter_value_requests",100,parameterValueRequestCallback);
  ros::Subscriber SetValueRequestSub
    = n.subscribe("set_value_requests",100,setValueRequestCallback);
  ros::Subscriber setParameterValueRequestSub
    = n.subscribe("set_parameter_value_requests",100,setParameterValueRequestCallback);

  ros::Rate loop_rate(100);

  string port = argv[1];
  unsigned long baud = atol(argv[2]);

  arduino_serial_ptr = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(250));
   if(arduino_serial_ptr->isOpen())
     if(DEBUG){cout << "serial port to arudino opened" << endl;}
   else
     if(DEBUG){cout << "failed to open serial port!" << endl;}

  string response = "";
  //each line read will have up to three tokens separated by a PARAM_CHAR OR ASSIGN_CHAR 
  stringstream tokens[3];
  char separatorChar[2];
  int tokenCounter = 0;
  int val;//used to hold any integer value sent from the arduino
  while (ros::ok())
  {

    if (arduino_serial_ptr->available() > 0){
	response = arduino_serial_ptr->readline(MAX_LINE_LENGTH,CARRIAGE_NEWLINE);

	tokenCounter = 0;
	separatorChar[0] = '0';
	separatorChar[1] = '0';
	tokens[0].str(string());
	tokens[1].str(string());
	tokens[2].str(string());
	for(i=0; i<response.size(); i++){
		if((response[i]==CARRIAGE_CHAR) || (response[i]==NEWLINE_CHAR)){break;}
		else if((response[i]==PARAM_CHAR) || (response[i]==ASSIGN_CHAR)){
			separatorChar[tokenCounter] = response[i];
			tokenCounter++;
			continue;
		}
		else{ tokens[tokenCounter] << response[i]; }
	}
	if (separatorChar[0] == ASSIGN_CHAR){//integer value is in the second token
		steev_driver::SteevValue msg;
		msg.id  = tokens[0].str();
		msg.val = atoi(tokens[1].str().c_str());
		valuePub.publish(msg);
	}
	else if (separatorChar[1] == ASSIGN_CHAR){//integer value is in the third token
		steev_driver::SteevParameterValue msg;
		msg.id      = tokens[0].str();
		msg.paramID = tokens[1].str();
		msg.val     = atoi(tokens[2].str().c_str());
		parameterValuePub.publish(msg);
	}

    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

