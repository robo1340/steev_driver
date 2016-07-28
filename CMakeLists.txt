cmake_minimum_required(VERSION 2.8.3)
project(steev_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
    ParameterValueRequest.msg
    SteevValue.msg
    ValueRequest.msg
    SteevParameterValue.msg
    SetParameterValueRequest.msg
    SetValueRequest.msg
    SteevSensor.msg
    SteevMechanicalOutputDevice.msg
    SteevFrame.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES steev_driver Sensor
  CATKIN_DEPENDS roscpp rospy std_msgs serial  message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(sensor_SRCS
	src/Sensor/Sensor.cpp
	include/steev_driver/Sensor.h
)
set(output_SRCS
	src/MechanicalOutputDevice/MechanicalOutputDevice.cpp
	include/steev_driver/MechanicalOutputDevice.h
)
#add_library(${PROJECT_NAME} ${sensor_SRCS})
add_library(Sensor ${sensor_SRCS})
add_library(MechanicalOutputDevice ${output_SRCS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(steev_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(serialTest src/serialTest.cpp)
target_link_libraries(serialTest ${catkin_LIBRARIES})
add_dependencies(serialTest steev_driver_generate_messages_cpp)

#add_executable(requester src/requester.cpp)
#target_link_libraries(requester ${catkin_LIBRARIES})
#add_dependencies(requester steev_driver_generate_messages_cpp)

add_executable(steevRXTX src/steevRXTX.cpp src/Sensor/Sensor.cpp src/MechanicalOutputDevice/MechanicalOutputDevice.cpp )
target_link_libraries(steevRXTX ${catkin_LIBRARIES})
add_dependencies(steevRXTX steev_driver_generate_messages_cpp)

add_executable(steevPublisher src/steevPublisher.cpp src/Sensor/Sensor.cpp src/MechanicalOutputDevice/MechanicalOutputDevice.cpp)
target_link_libraries(steevPublisher ${catkin_LIBRARIES})
add_dependencies(steevPublisher steev_driver_generate_messages_cpp)

