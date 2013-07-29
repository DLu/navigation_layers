cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_add_library(simple_layer src/simple_layer.cpp)
rosbuild_add_library(grid_layer src/grid_layer.cpp)

