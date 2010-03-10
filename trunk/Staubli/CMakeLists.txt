
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(soapC src/soapC.cpp)
rosbuild_add_library(Server0 src/soapCS8ServerV0Proxy.cpp)
rosbuild_add_library(Server1 src/soapCS8ServerV1Proxy.cpp)
rosbuild_add_library(Server3 src/soapCS8ServerV3Proxy.cpp)
rosbuild_add_library(stdsoap2 src/stdsoap2.cpp)
rosbuild_add_library(tx60l src/TX60L.cpp)

rosbuild_add_executable(staubli_controller src/controller.cpp)
target_link_libraries(staubli_controller soapC)
target_link_libraries(staubli_controller Server0)
target_link_libraries(staubli_controller Server1)
target_link_libraries(staubli_controller Server3)
target_link_libraries(staubli_controller stdsoap2)
target_link_libraries(staubli_controller tx60l)
