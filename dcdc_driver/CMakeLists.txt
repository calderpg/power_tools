cmake_minimum_required(VERSION 2.8.3)
project(dcdc_driver)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation sensor_msgs std_msgs power_msgs)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  power_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime sensor_msgs std_msgs power_msgs
  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Build the common library
add_library(usbtools src/usbtools.c)
# Build the command-line executable for monitoring the converter
add_executable(debugtool src/debugtool.c)
target_link_libraries(debugtool ${catkin_LIBRARIES} usbtools usb)
# Build the command-line executable for monitoring the converter
add_executable(voltagemonitor src/voltagemonitor.c)
target_link_libraries(voltagemonitor ${catkin_LIBRARIES} usbtools usb)
# Build the ROS node
add_executable(dcdc_node src/dcdc_node.cpp)
add_dependencies(dcdc_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dcdc_node ${catkin_LIBRARIES} usbtools usb)

