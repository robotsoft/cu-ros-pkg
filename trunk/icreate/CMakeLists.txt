cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rospack(irobot_create)   #deprecated
rosbuild_init()

rosbuild_add_library(irobot_create irobot_create.cpp)

rosbuild_add_executable(drive_in_circle drive_in_circle_turtlesim.cpp)
target_link_libraries(drive_in_circle irobot_create)

#rosbuild_add_executable(irobot_create_node irobot_create_node.cpp)
#target_link_libraries(irobot_create_node irobot_create acpi)

#rosbuild_add_executable(node_test node_test.cpp)
#target_link_libraries(node_test irobot_create)

#rosbuild_add_executable(random_explore random_explore.cpp)
#target_link_libraries(random_explore irobot_create)

#rosbuild_add_executable(battery_reader battery_reader.cpp)
#target_link_libraries(battery_reader irobot_create acpi)
