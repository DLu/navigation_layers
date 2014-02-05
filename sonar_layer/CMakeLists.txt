cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_add_library(${PROJECT_NAME} src/sonar_layer.cpp)


