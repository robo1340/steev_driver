#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>

#include "steev_driver/ValueRequest.h"

using std::string;
using std::exception;
using std::cout;
using std::cin;
using std::cerr;
using std::endl;
using std::vector;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver");
  ros::NodeHandle n;
  ros::Publisher sensorValueRequester = n.advertise<steev_driver::ValueRequest>("value_requests", 100);
  ros::Rate loop_rate(10);

  string request = "";
  std::stringstream ss;
  steev_driver::ValueRequest msg;
  while (ros::ok())
  {
//    ss << request << endl;
//    msg.data = ss.str();
    msg.id = "MIR";
    sensorValueRequester.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
