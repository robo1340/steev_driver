#ifndef STEEVDRIVERCONSTANTS_H
#define STEEVDRIVERCONSTANTS_H

namespace steev_constants
{

//sensor string
const std::string MAIN_IR = "MIR";
const std::string FRONT_LEFT_IR = "FLIR";
const std::string FRONT_RIGHT_IR = "FRIR";
const std::string LEFT_IR = "LIR";
const std::string RIGHT_IR = "RIR";
const std::string REAR_IR = "RRIR";
const std::string COMPASS = "CMP";

//sensor string parameter
const std::string PARAM_SENSOR_REFRESH_RATE = "REFR";
const std::string PARAM_SENSOR_STREAM_STATE = "STRM";
const std::string PARAM_SENSOR_ENABLE_STATE = "EN";

//output device string
const std::string PAN_SERVO = "PSVO";
const std::string TILT_SERVO = "TSVO";
const std::string FRONT_BACK_MOTOR_CHANNEL = "MFB";
const std::string RIGHT_LEFT_MOTOR_CHANNEL = "MLR";

//output device string parameter
const std::string PARAM_OUTPUT_DEVICE_INCREMENT = "INC";
const std::string PARAM_OUTPUT_DEVICE_DECREMENT = "DEC";
const std::string PARAM_OUTPUT_DEVICE_INCREMENT_AMOUNT = "SINC";

//special command string
const std::string CMD_LIST = "LIST";
const std::string CMD_SENSOR = "SENS";
const std::string CMD_OUTPUT_DEVICE = "OUTD";
const std::string CMD_VALUE_REQUEST = "VALU";
const std::string CMD_STOP = "STOP";

}
#endif

