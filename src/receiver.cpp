#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include "serial/serial.h"
using std::string;
using std::exception;
using std::cout;
using std::cin;
using std::cerr;
using std::endl;
using std::vector;

serial::Serial* arduino_serial_ptr;
string serialRequest;

void sensorRequestCallback(const std_msgs::String::ConstPtr& msg)
{
        cout << "request received"<<endl;
        serialRequest =  msg->data.c_str();
        serialRequest = serialRequest + "\r\n";
        arduino_serial_ptr->write(serialRequest);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver");
  ros::NodeHandle n;
  ros::Publisher sensorValuePublisher = n.advertise<std_msgs::String>("sensor_request_value", 100);
  ros::Subscriber sub = n.subscribe("sensor_requests", 100, sensorRequestCallback);
  ros::Rate loop_rate(100);

  string port = "/dev/ttyUSB0";
  unsigned long baud = 38400;
  arduino_serial_ptr = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
   if(arduino_serial_ptr->isOpen())
     cout << "serial port opened by receiver" << endl;
   else
     cout << "receiver failed to open serial port!" << endl;
  int inputBufferSize;
  string response = "";
  while (ros::ok())
  {

    if ((inputBufferSize = arduino_serial_ptr->available()) > 0){
        response = arduino_serial_ptr->read(inputBufferSize);
	std_msgs::String msg;
	msg.data = response;
	sensorValuePublisher.publish(msg);
	cout << "message from serial port received " << msg.data << endl;

    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
