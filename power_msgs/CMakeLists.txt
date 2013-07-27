cmake_minimum_required(VERSION 2.8.3)
project(power_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation sensor_msgs std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  PowerState.msg
)

add_service_files(
  DIRECTORY
  srv
  FILES
  SetOutput.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs std_msgs)
