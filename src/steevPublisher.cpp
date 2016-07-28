#include "ros/ros.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <vector>
#include <map>

#include "steev_driver/steevDriverConstants.h"

#include "steev_driver/SteevValue.h"
#include "steev_driver/SteevParameterValue.h"

#include "steev_driver/Sensor.h"
#include "steev_driver/MechanicalOutputDevice.h"
#include "steev_driver/SteevSensor.h"
#include "steev_driver/SteevMechanicalOutputDevice.h"
#include "steev_driver/SteevFrame.h"

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

//counter variable for for-loops
int i=0;

string id;
string pid;
int val;

bool updateFrame = false; //indicates that a new SteevFrame needs to be published

vector<Sensor> sensors;
vector<MechanicalOutputDevice> outputDevices;
#define numSensors 7
#define numOutputs 4

void valueCallback(const steev_driver::SteevValue::ConstPtr& msg)
{
	id = msg->id;
	val = msg->val;

	for(i=0; i<numSensors; i++){
		if (id.compare(sensors.at(i).getID()) == 0 ){//id.compare returns 0 when both strings are equal
			sensors.at(i).setValue(val);
			updateFrame = true;
			return;
		}
	}
	for(i=0; i<numOutputs; i++){
		if (id.compare(outputDevices.at(i).getID()) ==0 ){
			outputDevices.at(i).setValue(val);
			updateFrame = true;
			return;
		}
	}
}
void parameterValueCallback(const steev_driver::SteevParameterValue::ConstPtr& msg)
{
	id = msg->id;
	pid=msg->paramID;
	val = msg->val;

	for(i=0; i<numSensors; i++){
		if (id.compare(sensors.at(i).getID())==0){
			if (pid.compare(PARAM_SENSOR_REFRESH_RATE)==0){//val contains a new refresh rate
				sensors.at(i).setRefreshRate(val);
				updateFrame = true;
			}
			else if (pid.compare(PARAM_SENSOR_STREAM_STATE)==0){//val contains a stream enable state
				sensors.at(i).enableStream(val);
				updateFrame = true;
			}
			else if (pid.compare(PARAM_SENSOR_ENABLE_STATE)==0){//val contains an enable state
				sensors.at(i).setEnabled(val);
				updateFrame = true;
			}
			return;
		}
	}
	for(i=0; i<numOutputs; i++){
		if (id.compare(outputDevices.at(i).getID())==0){
			if (pid.compare(PARAM_OUTPUT_DEVICE_INCREMENT_AMOUNT)==0){//val contains a new inc amount
				outputDevices.at(i).setIncrementAmount(val);
				updateFrame = true;
				return;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////
//MAIN////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

  sensors.reserve(numSensors);
  sensors.push_back(Sensor(MAIN_IR));
  sensors.push_back(Sensor(FRONT_LEFT_IR));
  sensors.push_back(Sensor(FRONT_RIGHT_IR));
  sensors.push_back(Sensor(LEFT_IR));
  sensors.push_back(Sensor(RIGHT_IR));
  sensors.push_back(Sensor(REAR_IR));
  sensors.push_back(Sensor(COMPASS));

  //record the indice of each Sensor object for convenience
  int MIR = 0;
  int FLIR = 1;
  int FRIR = 2;
  int LIR = 3;
  int RIR = 4;
  int RRIR = 5;
  int CMP = 6;

  outputDevices.reserve(numOutputs);
  outputDevices.push_back(MechanicalOutputDevice(PAN_SERVO));
  outputDevices.push_back(MechanicalOutputDevice(TILT_SERVO));
  outputDevices.push_back(MechanicalOutputDevice(FRONT_BACK_MOTOR_CHANNEL));
  outputDevices.push_back(MechanicalOutputDevice(RIGHT_LEFT_MOTOR_CHANNEL));

  //record the indice of each MechanicalOutputDevice object for convenience
  int PSVO = 0;
  int TSVO = 1;
  int MFB = 2;
  int MLR = 3;

  ros::init(argc, argv, "frame publisher");
  ros::NodeHandle n;

  //everytime any steev_value or steev_parameter_value changes a new SteevFrame will be published
  ros::Publisher steevFramePublisher = n.advertise<steev_driver::SteevFrame>("steev_frame",100);

  //subscribe to the topics that the steevRXTX node publishes to
  ros::Subscriber valueSub = n.subscribe("steev_value",100,valueCallback);
  ros::Subscriber parameterValueSub = n.subscribe("steev_parameter_value",100,parameterValueCallback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    if (updateFrame = true) {
	updateFrame = false;

	steev_driver::SteevFrame msg; //create a new frame that will be published
	msg.mainIR.val           = sensors[MIR].getValue();
	msg.mainIR.refreshRate   = sensors[MIR].getRefreshRate();
	msg.mainIR.streamEnabled = sensors[MIR].isStreamEnabled();
	msg.mainIR.enabled       = sensors[MIR].isEnabled();

	msg.frontLeftIR.val           = sensors[FLIR].getValue();
	msg.frontLeftIR.refreshRate   = sensors[FLIR].getRefreshRate();
	msg.frontLeftIR.streamEnabled = sensors[FLIR].isStreamEnabled();
	msg.frontLeftIR.enabled       = sensors[FLIR].isEnabled();

	msg.frontRightIR.val           = sensors[FRIR].getValue();
	msg.frontRightIR.refreshRate   = sensors[FRIR].getRefreshRate();
	msg.frontRightIR.streamEnabled = sensors[FRIR].isStreamEnabled();
	msg.frontRightIR.enabled       = sensors[FRIR].isEnabled();

	msg.leftIR.val           = sensors[LIR].getValue();
	msg.leftIR.refreshRate   = sensors[LIR].getRefreshRate();
	msg.leftIR.streamEnabled = sensors[LIR].isStreamEnabled();
	msg.leftIR.enabled       = sensors[LIR].isEnabled();

	msg.rightIR.val           = sensors[RIR].getValue();
	msg.rightIR.refreshRate   = sensors[RIR].getRefreshRate();
	msg.rightIR.streamEnabled = sensors[RIR].isStreamEnabled();
	msg.rightIR.enabled       = sensors[RIR].isEnabled();

	msg.rearIR.val           = sensors[RRIR].getValue();
	msg.rearIR.refreshRate   = sensors[RRIR].getRefreshRate();
	msg.rearIR.streamEnabled = sensors[RRIR].isStreamEnabled();
	msg.rearIR.enabled       = sensors[RRIR].isEnabled();

	msg.compass.val           = sensors[CMP].getValue();
	msg.compass.refreshRate   = sensors[CMP].getRefreshRate();
	msg.compass.streamEnabled = sensors[CMP].isStreamEnabled();
	msg.compass.enabled       = sensors[CMP].isEnabled();

	msg.panServo.val             = outputDevices[PSVO].getValue();
	msg.panServo.incrementAmount = outputDevices[PSVO].getIncrementAmount();

	msg.tiltServo.val             = outputDevices[TSVO].getValue();
	msg.tiltServo.incrementAmount = outputDevices[TSVO].getIncrementAmount();

	msg.motorLinear.val             = outputDevices[MFB].getValue();
	msg.motorLinear.incrementAmount = outputDevices[MFB].getIncrementAmount();

	msg.motorTurn.val             = outputDevices[MLR].getValue();
	msg.motorTurn.incrementAmount = outputDevices[MLR].getIncrementAmount();

	steevFramePublisher.publish(msg);

    }
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

