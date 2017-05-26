cmake_minimum_required(VERSION 3.7)
project(teleop_vehicle_joy)

set(CMAKE_CXX_STANDARD 14)
include_directories(include D:/Programs/ROS/include)

set(SOURCE_FILES src/teleop_node.cpp)
add_executable(teleop_vehicle_joy ${SOURCE_FILES})