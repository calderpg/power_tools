cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#build the common library
rosbuild_add_library(usbtools src/usbtools.c)
#build the command-line executable for setting & getting data
rosbuild_add_executable(debugtool src/debugtool.c)
target_link_libraries(debugtool usbtools usb)
#build the command-line executable for monitoring the converter
rosbuild_add_executable(voltagemonitor src/voltagemonitor.c)
target_link_libraries(voltagemonitor usbtools usb)
#build the ROS node
rosbuild_add_executable(dcdc_node src/dcdc_node.cpp)
target_link_libraries(dcdc_node usbtools usb)
