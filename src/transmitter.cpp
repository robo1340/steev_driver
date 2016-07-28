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

serial::Serial*  my_serial_ptr;
string serialRequest;

void sensorRequestCallback(const std_msgs::String::ConstPtr& msg)
{
	cout << "request received"<<endl;
	serialRequest =  msg->data.c_str();
	serialRequest = serialRequest + "\r\n";
	my_serial_ptr->write(serialRequest);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("sensor_requests", 100, sensorRequestCallback);

  string port = "/dev/ttyUSB0";
  unsigned long baud = 38400;
 my_serial_ptr = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
   if(my_serial_ptr->isOpen())
     cout << "serial port opened by transmitter" << endl;
   else
     cout << "transmitter failed to open serial port!" << endl;


  ros::spin();

  return 0;
}
